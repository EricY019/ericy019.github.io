---
title: "Push Platform Architecture: Real-Time Data Delivery at Scale"
categories:
  - Blog
tags:
  - Push Architecture
  - Distributed Systems
---

This blog summarizes the architecture of a real-time push platform designed to deliver market data streams to large numbers of concurrent clients via WebSocket. The platform is built around a clean separation of concerns across data ingestion, message processing, subscription management, and delivery.

---

## Module Organization

The codebase is organized into four layers:

| Module | Responsibility |
|--------|---------------|
| `repository` | Manages Kafka consumer lifecycle and Redis sessions |
| `common` | Shared utilities: Netty bootstrapping, protobuf codecs, level2 order book, etc. |
| `biz` | Business-level channel logic: depth aggregation, p2p fanout, channel-specific processing |
| `service` | Orchestration layer; assembles message routing and network topology |
| `web/app` | Entry-point modules: `inner-public` (unauthenticated) and `inner-private` (authenticated) |

The `service` layer references both `inner-public` and `inner-private` modules. This split allows the platform to serve different client types — public market data consumers vs. credentialed private-channel clients — through separate pipeline configurations while reusing the same core processing infrastructure.

---

## Channel Types

Three distinct channel types are supported:

- **Public channels**: Publicly accessible market data such as tickers, trades, and order book snapshots. Clients connect without authentication.
- **Private channels**: User-specific streams (e.g., account updates, order fills) that require JWT-based login before subscription.
- **Depth channels**: Dedicated high-frequency order book (L2) channels that use a specialized byte-level data consumer and a replay buffer to ensure snapshot consistency.

---

## Middleware Stack

The platform is built on a carefully selected set of middleware components, each chosen for a specific technical reason:

### Kafka
Kafka serves as the primary ingestion bus between upstream data producers and the push platform. It decouples the data generation lifecycle from the delivery layer, provides natural backpressure handling, and enables consumer-group-based parallelism. The push platform runs Kafka consumers per channel partition.

### Aeron
Aeron is used for ultra-low-latency intra-service message transport. Unlike TCP-based alternatives, Aeron operates over UDP with a lock-free IPC mechanism, achieving sub-microsecond latencies. It is used on paths where even Kafka's minimal overhead is too much.

### gRPC
gRPC is used for private channel delivery in service-to-service communication. Its bidirectional streaming model fits naturally into a reactive push pipeline: the upstream service pushes data via `onNext()` on a remote stream observer, which flows through to the client's channel. Strongly-typed protobuf contracts ensure correctness at the interface boundary.

### Redis
Redis is used as a session store. When a client authenticates on an inner-private connection, the resulting `Authentication` object is stored and associated with the session. It is also used for lightweight caching of channel metadata.

### Netty
Netty provides the async, non-blocking I/O framework. All WebSocket connections are managed by Netty. The platform leverages the `ChannelPipeline` abstraction to compose modular, reusable handlers in a clean chain.

### LMAX Disruptor (Ring Buffer)
The Disruptor is used as the core inter-thread message passing mechanism between the Kafka consumer threads and the session delivery threads. It is a lock-free single-writer ring buffer — avoiding the mutex contention that would otherwise arise in a high-frequency fanout scenario. Publishers `tryClaim` a slot, write the event, and `publish`; consumers call `handleEventsWithMsg` asynchronously.

---

## Netty Channel Pipeline


### Public Pipeline
Handles unauthenticated public channel clients. The handler chain performs path validation, idle connection detection, WebSocket ping/pong health checks, per-connection rate limiting, and request decoding — before handing control to the subscribe logic.

### Private Pipeline
Extends the public pipeline with two additional stages: a **login handler** that performs JWT-based authentication (using a reactive authentication manager and a token verification service), and a **subscribe handler** that only processes `SUBSCRIBE` frames after the session has been successfully authenticated. The session's `Authentication` object is bound to the channel context upon successful login.

The key design insight here is that both pipelines share the same core handlers for connection lifecycle and rate limiting, but diverge only at the authentication boundary. This makes it easy to enforce consistent connection-level policies while maintaining separate security postures for public vs. private clients.

---

## Subscription Flow

### Authentication (Private)
When a private-channel client connects, it must first send a login request. The login handler validates the JWT token reactively (non-blocking) and binds the resulting session identity to the connection. Only after this handshake can the client issue `SUBSCRIBE` requests.

### Public Channel Subscription
Upon receiving a subscribe request, the platform parses the channel name and subscription rules to identify the matching channel configuration. It immediately returns a success acknowledgment to the client, then registers the subscriber into a **fanout handler** — a per-channel structure that manages all active subscriptions for that channel. Each fanout handler groups subscribers by encoder type to avoid redundant serialization.

Subscription state is tracked through a simple state machine (`INIT → WAITING → RUNNING`): a new subscription starts by claiming a slot in the Disruptor ring buffer, waits for the initial data push, and transitions to a steady-state running mode served by partition-aligned repository handlers.

### Private Channel Subscription
Private subscriptions follow the same parse-and-fanout pattern as public ones, but with two distinct backend paths depending on the data source:
- **Kafka-backed**: The subscription proxy mirrors the public flow, routing through the internal Kafka-based repository.
- **gRPC-backed**: A persistent bidirectional gRPC stream is established to the upstream service. The remote service pushes data via stream callbacks, which flow through the session's subscriber chain to the client.

### Depth Channel Subscription
Depth subscriptions go through a specialized proxy that resolves the instrument ID and depth level from the channel name. The fanout layer aggregates all subscribers for the same depth level together, allowing the system to compute and encode the order book snapshot once and broadcast it to all matching sessions — avoiding redundant L2 computation per subscriber.

---

## Push Flow

### Public Channel Push
Data flows from Kafka into a `DataStreamRepository`, which fans out to multi-partition handlers. Each partition handler publishes events into the **Disruptor ring buffer** asynchronously. On the consumer side, the async handler dispatches data through the fanout tree — from multi-channel handler → single-channel handler → per-subscriber observer — and ultimately batches writes to the Netty channel via a `BatchFlusher`.

The multi-partition design ensures that ordering is preserved within a single channel while allowing parallelism across channels.

### Private Channel Push
- **Kafka path**: Structurally identical to the public path. The private subscription proxy acts as a `SubObserver`, receiving data from the Disruptor and flushing to the client channel.
- **gRPC path**: The upstream service pushes data via gRPC stream callbacks. Each callback is received by the session proxy, forwarded to the subscriber observer, and batch-flushed to the client channel — no Disruptor involvement on this path.

### Depth Channel Push
Depth data follows a dedicated high-throughput pipeline:

1. Upstream byte-level messages arrive through a **ParagonReplayBuffer**, which ensures snapshot/delta ordering and supports replay on reconnect.
2. A dedicated `DepthDataConsumer` processes raw byte buffers and publishes events into the Disruptor ring buffer via multi-partition depth handlers.
3. On the consumer side, the async handler maintains an in-memory **level2 order book** per instrument, updated by three event types:
   - **Ticker** events: update the book's timestamp and trigger delivery to all depth channel subscribers.
   - **Snapshot** events: rebuild the full order book state.
   - **Trade/AccAndTrade** events: push incremental depth line and trade updates to subscribers.

The level2 book is maintained entirely in-process, eliminating the need for a remote state store on the critical delivery path.


---

## Design Rationale

### Why Disruptor instead of a blocking queue?
A traditional `BlockingQueue` introduces lock contention under high producer throughput. The LMAX Disruptor uses a lock-free ring buffer with sequence-based coordination, eliminating cache-line invalidation across threads. For a system processing millions of market data events per second, this is a critical performance optimization.

### Why Kafka for ingestion?
Kafka provides durable, ordered, partitioned log semantics. The push platform partitions channels across Kafka partitions, enabling horizontal fan-out by simply adding consumer instances. Kafka also provides backpressure: if downstream clients are slow, messages accumulate in the log rather than causing memory pressure in the push process.

### Why Aeron for internal transport?
Where Kafka's persistence overhead is unnecessary (intra-JVM or intra-host IPC), Aeron provides a shared-memory transport with lock-free SPSC queues. This enables latencies in the hundreds of nanoseconds range on critical hot paths.

### Why gRPC for private channels?
Private channel data is often typed, complex, and bidirectional. gRPC's bidirectional streaming over HTTP/2 fits this model naturally. The `onNext()` API maps directly to a reactive push model, and protobuf contracts provide a strongly-typed, version-tolerant interface between services.

### Why Netty pipeline composition?
The `ChannelPipeline` pattern cleanly separates cross-cutting concerns — authentication, rate limiting, codec, idle detection — from business logic. Each handler is independently testable and replaceable. The divergence between inner-public and inner-private pipelines is handled by including/excluding specific handlers at startup, without changing any core logic.

### Why Fanout abstraction?
A single Kafka message may need to be delivered to thousands of concurrent subscribers of the same channel. The `FanoutSubChannelProxy` centralizes this one-to-many dispatch, avoiding duplicate deserialization per subscriber. Each `oneChannelOneEncoderHandler` encodes once per encoder type and writes to all matching sessions.

### Why Multi-partition strategy?
Channels are sharded across Kafka partitions. `MultiPartitionRepositoryHandler` maps channel IDs to partitions, enabling parallel consumption and ensuring ordering is maintained within a single channel while maximizing throughput across channels.

---

## Summary

The push platform achieves high-throughput, low-latency real-time delivery through a layered architecture:

- **Ingestion**: Kafka (durable, partitioned, backpressure-safe)
- **Internal transport**: Aeron (sub-microsecond IPC) and gRPC (typed bidirectional streaming)
- **Session management**: Redis (auth tokens, session state)
- **Async dispatch**: LMAX Disruptor (lock-free ring buffer, nanosecond-scale inter-thread handoff)
- **Network I/O**: Netty (async NIO, composable pipeline)
- **Business patterns**: Fanout, multi-partition sharding, state machine (INIT/WAITING/RUNNING), ParagonReplayBuffer for depth consistency

Each middleware choice directly addresses a bottleneck or correctness concern: Disruptor for contention-free async processing, Kafka for reliable decoupled ingestion, Netty for scalable connection management, and Fanout for efficient one-to-many dispatch. Together, they form a system capable of sustaining millions of concurrent channel subscriptions with consistently low delivery latency.
