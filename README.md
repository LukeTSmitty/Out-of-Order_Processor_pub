# VadaPav: A 2-Way Superscalar Out-of-Order RV32IM Processor

**ECE 411 Final Project | University of Illinois at Urbana-Champaign**
**Team VadaPav:** Ashmit Dutta, Luke Smith, David Mun

---
Due to academic integrity guidelines, full implementation of this processor is kept hidden. Detailed walkthrough, architecture discussion, and code review available by request or during an interview. Feel free to message me on Linkedin (https://www.linkedin.com/in/luke-smith-500730377/)
## Overview

`mp_ooo` is a synthesizable, out-of-order RISC-V processor implementing the RV32IM ISA (excluding `FENCE*`, `ECALL`, `EBREAK`, and `CSRR`). The design uses **Explicit Register Renaming (ERR)** with Tomasulo-style dynamic scheduling, a split L1 instruction/data cache hierarchy backed by a banked-burst DRAM model, and a suite of advanced microarchitectural features targeting the ECE 411 class design competition.

The final submitted processor averages **0.77 IPC** across six benchmarks (peak **1.06 IPC** on `compression`) and achieves a geometric-mean PD⁴ of approximately **253** (with area penalty) — a **3.6× improvement** over the staff baseline. With the split LSQ enabled, the processor reaches a peak of **1.325 IPC** on `compression`.

---

## Motivation

Out-of-order execution is the dominant performance paradigm in modern processors. This project served as a ground-up implementation of those techniques. The course's design competition added urgency to optimize for PD⁴ (Power × Delay⁴), pushing the team to analyze IPC, area, power, and timing together rather than in isolation.

A key early decision was to design for **flexibility from the start**: parameterizing ROB depth, PRF size, and branch predictor table dimensions in `types.sv`. This allowed the tournament branch predictor, RAS, BTB, split LSQ, and 2-wide superscalar execution lane to be added or tuned late in the project without restructuring core datapaths.

---

## Architecture

### Front End

| Component | Details |
|---|---|
| **PC & Fetch** | 2-wide fetch; primary + victim linebuffer (256-bit, 2 × 32 B slots) |
| **I-Cache** | 3-way, 64 sets, 32 B lines (6 KB); 256-bit cacheline adapter for DRAM bursts |
| **Instruction Queue** | 12-entry parameterized circular FIFO; 2-enqueue / 2-dequeue per cycle |
| **Decode** | Stateless combinational stage; classifies RV32IM instructions for dispatch |
| **Branch Prediction** | McFarling tournament predictor (bimodal + gshare + chooser, 256 entries/table); BTB (8-entry direct-mapped); RAS (4-entry) |

The linebuffer eliminates the every-other-cycle stall from `mp_cache` response timing, enabling up to 1 IPC fetch on sequential cacheline accesses. The 2-wide parallelization of fetch and decode sustains two micro-ops per cycle into dispatch.

### Backend

| Component | Details |
|---|---|
| **RAT** | 32 entries × 6-bit physical index; bulk single-cycle restore from RRF on flush |
| **PRF** | 48 physical registers |
| **Free List** | 16 free registers (bitmap; two lowest-numbered allocated combinationally per cycle) |
| **ROB** | 16-entry circular queue; 2-wide alloc/commit |
| **RS_ALU** | 8 entries; dual ALU (ALU0 combinational + ctrl, ALU1 pipelined) |
| **RS_MUL** | 1 entry; 4-cycle pipelined multiplier |
| **RS_DIV** | 1 entry; 32-cycle divider |
| **RS_MEM** | 10 entries; AGU (combinational address) |
| **CDB** | 2 slots; priority: LOAD > ALU0 > ALU1 > MUL > DIV |
| **D-Cache** | 4-way, 16 sets, 32 B lines (2 KB) |

**Register Renaming (ERR):** The RAT maps architectural registers to physical registers. The RRF (Retirement Register File) is a committed-state snapshot of the RAT used for single-cycle flush recovery. The free list recycles `rd_old_phys` at commit. Reservation stations carry physical register indices and ready bits; operands are read from the PRF at issue, not stored in the RS entries. This matches how modern OOO processors operate and simplifies flush recovery and functional-unit expansion.

**Dispatch** checks ROB space, free-list availability, and RS vacancy before allocating. It also snoops the CDB at dispatch to set ready bits for values that complete in the same cycle.

**Commit** retires up to 2 ROB entries per cycle in program order. On a branch mispredict, commit asserts a flush: the RRF is bulk-copied into the RAT in one cycle, the ROB/RS/free-list states are cleared, and the frontend is redirected to the correct PC.

---

## Advanced Features

### 1. 2-Way Superscalar (10 pts)

The entire pipeline was widened front-to-back to sustain two instructions per cycle:

- **Fetch:** Primary + victim linebuffer; extracts two aligned instructions per cycle on a hit.
- **Decode/Dispatch:** Processes two micro-ops per cycle; allocates two ROB entries, two PRF destinations, and two RS slots in a single cycle. Handles intra-pair RAW hazards (the second instruction may depend on the first) by forwarding the physical tag or serializing as needed.
- **CDB:** Widened to 2 slots with fixed priority to avoid arbitration logic.
- **Commit:** Retires up to 2 ROB entries per cycle; suppresses the second retire on flush or if the second entry is not yet ready.

**Performance impact (representative):**

| Benchmark | IPC Before | IPC After | PD⁴ Before | PD⁴ After |
|---|---|---|---|---|
| compression | 0.776 | 1.045 | 82.28 | 25.06 |
| fft | 0.751 | 0.962 | 4868 | 1811 |
| mergesort | 0.607 | 0.740 | 342.4 | 155.3 |
| aes_sha | 0.340 | 0.359 | 22700 | 18256 |

`compression` gains the most (+35%) due to tight computational loops with high ILP. `aes_sha` gains least (+6%) because its 28.5% branch misprediction rate wastes the second issue slot via frequent flushes.

### 2. Split Load/Store Queue (6 pts)

The original design tracked loads and stores entirely through the ROB with a post-commit store buffer, which overloaded the ROB with memory ordering decisions and forced all loads to wait behind any pending stores.

The split LSQ (`split_lsq.sv`) decouples this into:

- **Load Queue (LQ):** Tracks loads from dispatch through address resolution and cache access; retires in program order. Non-blocking: a load can issue out-of-order past older resolved stores at different addresses.
- **Store Queue (SQ):** Tracks stores from dispatch through address resolution, commit, drain to D-cache, and removal. The ROB marks a store committed; it stays in the SQ until the D-cache write completes.

**Store-to-load forwarding:** If the youngest matching older store fully covers the load's byte mask, the LQ forwards the value directly from the SQ without a cache access. Partial overlaps and unknown-address stores stall the load conservatively.

**Performance impact (representative):**

| Benchmark | IPC Before | IPC After |
|---|---|---|
| fft | 0.9532 | 0.9977 |
| aes_sha | 0.5786 | 0.6627 |
| image | 0.3971 | 0.4049 |
| compression | 1.0561 | 1.3251 |

`fft` forwarded 10,251 loads; `aes_sha` forwarded 4,618 — explaining their largest gains.

### 3. Tournament Branch Predictor + BTB + RAS (5 pts + 2 pts)

**Tournament Predictor (`bp_tournament.sv`):** A McFarling-style predictor combining:
- **Bimodal table** (256 entries, 2-bit saturating counters, indexed by PC)
- **GShare table** (256 entries, indexed by `pc_index ⊕ GHR` with 14-bit GHR)
- **Chooser table** (256 entries, 2-bit counters selecting between bimodal and gshare; trains only when the two predictors disagree)

To reduce read-side switching activity (the dominant power term), the bimodal and chooser tables share a PC index and are merged into a single 4-bit array (`bc_bank`), split into 4 banks of N/4 entries. Inactive bank offsets are gated to zero, achieving ~75% reduction in mux-tree toggling.

**Branch Target Buffer (`btb.sv`):** 8-entry direct-mapped, tagged on PC; queried combinationally at fetch; trained synchronously at commit on taken `JALR` instructions that are not returns (returns are handled by the RAS).

**Return Address Stack (`ras.sv`):** 4-entry stack with separate fetch-time and architectural stack pointers. Calls push `pc+4`; returns pop. The fetch pointer snaps back to the committed pointer on a flush.

**Performance impact of dynamic branch prediction:**

| Benchmark | IPC Before | IPC After | PD⁴ Improvement |
|---|---|---|---|
| aes_sha | 0.349 | 0.579 | 7.6× |
| coremark_im | 0.690 | 0.822 | 2× |
| compression | 1.050 | 1.056 | ~same |

`aes_sha` benefits most (66% IPC gain) due to its 28.5% misprediction rate under static predict-not-taken; the tournament predictor dramatically reduces this. Compute-dominated benchmarks like `compression` and `fft` see minimal change because they have few branches.

---

## Microarchitectural Structure Sizes

| Structure | Size | Rationale |
|---|---|---|
| I-Cache | 3-way, 64 sets, 32 B lines (6 KB) | Reduced from 4-way to meet area; 32 B lines match DRAM burst width |
| Linebuffer | 2 slots × 32 B | Added second slot for 2-wide superscalar fetch |
| IQ | 12 entries, 2-enq/2-deq | Absorbs fetch/decode rate mismatches across cache-miss bubbles |
| BTB | 8 entries | Sized for area; targets indirect `JALR` only (returns go to RAS) |
| RAS | 4 entries | Covers stack depth in CoreMark/Dhrystone-like benchmarks |
| RAT | 32 × 6-bit phys_idx | Matches RV32I architectural register count |
| PRF | 48 physical registers | Dropped from 64 after area analysis showed upper 16 rarely used |
| ROB | 16 entries | Below 32 by design; wider ROB hit area/timing without proportional IPC gain |
| ALU RS | 8 entries | Sized down from 12 via perf counters; `rs_alu_full` was a minor stall contributor |
| MEM RS | 10 entries | Largest RS; accommodates many in-flight loads (e.g., mergesort) |
| CDB | 2 slots | Matches 2-wide commit / 2-wide ROB writeback / 2 PRF write ports |
| D-Cache | 4-way, 16 sets, 32 B lines (2 KB) | 32 B lines match DRAM burst geometry |

---

## Performance Results

Synthesized at **500 MHz** (2.0 ns clock period); area = **304,648 µm²** (area penalty factor ≈ 1.066 above the 300,000 µm² threshold). Power measured via SAIF-based switching activity.

**Final results vs. staff baseline:**

| Benchmark | Baseline IPC | Ours IPC | Baseline PD⁴ | Ours PD⁴ |
|---|---|---|---|---|
| coremark_im | 0.398 | **0.822** | 87.187 | **12.62** |
| aes_sha | 0.309 | **0.579** | 11826.027 | **2577** |
| compression | 0.484 | **1.056** | 188.395 | **21.90** |
| fft | 0.508 | **0.953** | 8229.957 | **1754** |
| mergesort | 0.431 | **0.834** | 471.743 | **88.89** |
| image | 0.376 | **0.397** | 750.796 | 1603 |

The design placed in the **top 10** of the ECE 411 course competition, with some metrics reaching 2nd place. It passes lint and synthesis without warnings, and matches Spike and RVFI on every released benchmark.

---

## Development Milestones

| Checkpoint | Scope |
|---|---|
| **CP1** | Processor frame and front-end memory path; instruction fetch from DRAM; linebuffer; 256-bit cacheline adapter; instruction queue |
| **CP2** | Full OOO execution backend for ALU/MUL/DIV instructions; RAT, PRF, free list, reservation stations, ROB, CDB; commit and in-order retirement |
| **CP3** | Complete RV32IM baseline: memory instructions (AGU, LB/LH/LW/LBU/LHU/SB/SH/SW), D-cache integration, I/D-cache arbitration, branch/JAL/JALR/AUIPC, mispredict recovery |
| **Advanced** | 2-way superscalar, split LSQ with OOO loads and store-to-load forwarding, tournament branch predictor + BTB + RAS |

---

## Team Organization

Work was divided by **functional axis** rather than frontend/backend split. Each member owned a specific datapath function end-to-end across the full pipeline. This avoided the integration cliff that typically appears when independently developed frontend and backend halves first meet. Cross-boundary decisions were always collaborative, keeping all three members able to debug any part of the codebase.

