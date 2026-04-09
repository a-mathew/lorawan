# LoRaWAN ns-3 module (with Class C support)

[![CI](https://github.com/signetlabdei/lorawan/actions/workflows/per-commit.yml/badge.svg)](https://github.com/signetlabdei/lorawan/actions)

This is an [ns-3](https://www.nsnam.org "ns-3 Website") module for simulating
[LoRaWAN](https://lora-alliance.org/about-lorawan "LoRa Alliance") networks,
forked from [signetlabdei/lorawan](https://github.com/signetlabdei/lorawan).

This fork adds **Class C end device support** compliant with LoRaWAN L2 v1.0.4 (Section 15),
including a continuous receive window (RXC), server-initiated downlinks, and a realistic
channel example with the 3-layer propagation model used in the literature.

Quick links:

* [Upstream Module Documentation](https://signetlabdei.github.io/lorawan/models/build/html/lorawan.html)
* [Upstream API Reference](https://signetlabdei.github.io/lorawan/html/d5/d00/group__lorawan.html)

---

## What's new in this fork

The upstream module only supports Class A devices. This fork extends it with a full
Class C implementation while preserving backward compatibility with all existing
Class A functionality.

### Class C end device MAC

The core addition is `ClassCEndDeviceLorawanMac`, which extends the existing
`ClassAEndDeviceLorawanMac` with a continuous receive window (RXC) that stays
open whenever the device is not transmitting or inside a Class A RX1/RX2 window.

Per the spec, the timing within a single uplink cycle looks like this:

```
  |<-- TX -->|<-- RXC -->|<-- RX1 -->|<-- RXC -->|<-- RX2 -->|<--- RXC (continuous) --->
             ^           ^           ^           ^           ^
             |           |           |           |           |
        end of TX     DELAY1      close       DELAY2      close
                                   RX1                     RX2
```

Key behaviors:
- RXC uses the same frequency and data rate as RX2 (default EU868: 869.525 MHz, DR0/SF12).
- If a downlink is being demodulated on RXC when RX1 or RX2 needs to open, the RXC
  demodulation is terminated and the Class A window takes priority.
- If a valid downlink is received in RX1, RX2 is skipped and RXC reopens immediately.
- `RxParamSetupReq` MAC commands update both RX2 and RXC parameters together.

### Application server and downlink enqueueing

The `NetworkServer` now exposes an Application Server interface for pushing
downlinks to end devices:

```cpp
// Forward uplink payloads to your application logic
nsApp->SetUplinkForwardCallback(MakeCallback(&MyApp::OnUplink, myApp));

// Enqueue a downlink for a Class C device (sent via RX2/RXC)
nsApp->EnqueueDownlink(deviceAddress, payload);
```

A standalone `LoraApplicationServer` application is also provided for
scenarios where the AS runs on a separate node.

### Realistic channel example

`class-c-realistic-channel-example` demonstrates Class C under the 3-layer
propagation model used in the published literature (Magrin et al., IEEE IoT
Journal 2019):

1. **Log-distance path loss** (n = 3.76, d0 = 1 m, PL(d0) = 7.7 dB)
2. **Correlated shadowing** (sigma = 4 dB, correlation distance = 110 m)
3. **Building penetration loss** (3GPP TR 45.820 model)

```bash
# Basic channel (log-distance only)
./ns3 run class-c-realistic-channel-example

# Full realistic channel
./ns3 run "class-c-realistic-channel-example --realisticChannel=true --nDevices=100"
```

---

## Architecture

```
 +-----------------------------------------------------------------+
 |                        LoRaWAN Network                          |
 |                                                                 |
 |  End Devices                Gateway             Network Server  |
 |  +-----------------+    +------------+    +-------------------+ |
 |  | PeriodicSender  |    |            |    |   NetworkServer   | |
 |  +-----------------+    |  Forwarder |    |   (Application)   | |
 |  | EndDeviceMac    |    |            |    +--------+----------+ |
 |  |  +------------+ |    +------+-----+             |            |
 |  |  | Class A    | |           |              +----+-----+      |
 |  |  |   MAC      | |     P2P Link            | Network  |      |
 |  |  +-----+------+ |           |              | Status   |      |
 |  |  | Class C    | |           |              +----------+      |
 |  |  |   MAC      | |           |              | Network  |      |
 |  |  | +--------+ | |           |              | Scheduler|      |
 |  |  | |  RXC   | | |           |              +----------+      |
 |  |  | |(contin)| | |           |              | Network  |      |
 |  |  | +--------+ | |           |              |Controller|      |
 |  |  +------------+ |           |              +----------+      |
 |  | EndDevicePhy    |    | GatewayPhy  |                         |
 |  +---------+-------+    +------+------+                         |
 |            |                   |                                |
 |            +------- LoRa Channel (868 MHz) -----+               |
 |                   LogDistance -> Shadowing -> BuildingLoss       |
 +-----------------------------------------------------------------+

 Class C MAC State Machine
 =========================

              +-----+
              | TX  |
              +--+--+
                 |
                 v
         +-------+-------+
         | RXC (gap 1)   |  <-- listening on RX2 freq/DR
         +-------+-------+
                 |
                 v  (RECEIVE_DELAY1)
              +--+--+
              | RX1 |  <-- uplink channel freq, RX1 DR
              +--+--+
                 |
                 v
         +-------+-------+
         | RXC (gap 2)   |
         +-------+-------+
                 |
                 v  (RECEIVE_DELAY2)
              +--+--+
              | RX2 |  <-- 869.525 MHz, DR0
              +--+--+
                 |
                 v
         +-------+-----------+
         | RXC (continuous)  |  <-- stays open until next TX
         +-------------------+
```

### File layout

```
model/
  class-c-end-device-lorawan-mac.cc/h  -- Class C MAC (extends Class A)
  lora-application-server.cc/h         -- Application Server model
  network-server.cc/h                  -- Modified: downlink API
  end-device-lorawan-mac.cc            -- Modified: FIFO postponed TX

helper/
  lorawan-mac-helper.cc/h              -- Modified: ED_C device type
  lora-application-server-helper.cc/h  -- AS helper
  network-server-helper.cc/h           -- Modified: Class C registration

examples/
  class-c-example.cc                   -- Basic Class C simulation
  class-c-realistic-channel-example.cc -- 3-layer channel model
  app-server-verify-example.cc         -- Application server demo
```

---

## Getting started

### Prerequisites

To run simulations using this module, you first need to install ns-3. If you are on Ubuntu/Debian/Mint, you can install the minimal required packages as follows:

```bash
sudo apt install g++ python3 cmake ninja-build git ccache
```

Otherwise please directly refer to the [prerequisites section of the ns-3 installation page](https://www.nsnam.org/wiki/Installation#Prerequisites).

> Note: While the `ccache` package is not strictly required, it is highly recommended. It can significantly enhance future compilation times by saving tens of minutes, albeit with a higher disk space cost of approximately 5GB. This disk space usage can be eventually reduced through a setting.

Then, you need to:

1. Clone the main ns-3 codebase,
1. Clone this repository inside the `src` directory therein, and
1. Checkout the correct ns-3 version supported by this module.

```bash
git clone https://gitlab.com/nsnam/ns-3-dev.git && cd ns-3-dev &&
git clone https://github.com/a-mathew/lorawan src/lorawan &&
cd src/lorawan && git checkout class-c && cd ../.. &&
tag=$(< src/lorawan/NS3-VERSION) && tag=${tag#release } && git checkout $tag -b $tag
```

### Compilation

Configure and build (from the `ns-3-dev` folder):

```bash
./ns3 configure --enable-tests --enable-examples &&
./ns3 build
```

To only build the lorawan module (faster):

```bash
./ns3 clean &&
./ns3 configure --enable-tests --enable-examples --enable-modules lorawan &&
./ns3 build
```

Verify tests pass:

```bash
./test.py
```

## Usage examples

The module includes the following examples:

* `simple-network-example`
* `complete-network-example`
* `network-server-example`
* `adr-example`
* `aloha-throughput`
* `frame-counter-update`
* `lorawan-energy-model-example`
* `parallel-reception-example`
* **`class-c-example`** -- Class C end devices with basic channel
* **`class-c-realistic-channel-example`** -- Class C with realistic propagation
* **`app-server-verify-example`** -- Application server downlink demo

Examples can be run via the `./ns3 run example-name` command (refer to `./ns3 run --help` for more options).

### Running the Class C examples

```bash
# Basic Class C (log-distance path loss only)
./ns3 run class-c-example

# Realistic channel with 50 devices (default)
./ns3 run "class-c-realistic-channel-example --realisticChannel=true"

# Scale up: 200 devices, 3 km radius
./ns3 run "class-c-realistic-channel-example --realisticChannel=true --nDevices=200 --radius=3000"
```

Sample output from the realistic channel example:

```
=== Class C Realistic Channel Simulation Results ===
Channel model: Realistic (LogDist + Shadowing + Building)
Devices: 50  Gateways: 1  Radius: 3000 m  Duration: 600 s

--- MAC-level (global) ---
Sent Received: 500 285

--- PHY-level (per gateway) ---
TotPktOnGW RecByGW IntfOnGW NoMoreDem UnderSens LostBcTx: 500 0 0 0 0 0
```

## Propagation models

The module ships with three propagation loss models that can be chained together
for varying levels of realism:

| Layer | Model | Parameters | Reference |
|-------|-------|-----------|-----------|
| 1 | Log-distance path loss | n=3.76, d0=1m, PL(d0)=7.7 dB | Standard RF propagation |
| 2 | Correlated shadowing | sigma=4 dB, d_corr=110 m | Schlegel et al. (IEEE TVCG 2012) |
| 3 | Building penetration | 4-23 dB external wall loss | 3GPP TR 45.820 |

Interference between concurrent transmissions is handled by the `LoraInterferenceHelper`
using the Goursaud collision matrix (co-SF capture threshold: 6 dB SIR).

Receiver sensitivity per spreading factor (from SX1272/SX1301 datasheets):

| | SF7 | SF8 | SF9 | SF10 | SF11 | SF12 |
|---|---|---|---|---|---|---|
| End device | -124 | -127 | -130 | -133 | -135 | -137 dBm |
| Gateway | -130 | -132.5 | -135 | -137.5 | -140 | -142.5 dBm |

## Documentation

* [Upstream Simulation Model Overview](https://signetlabdei.github.io/lorawan/models/build/html/lorawan.html): A description of the foundational models of this module (source file located at `doc/lorawan.rst`).
* [Upstream API Documentation](https://signetlabdei.github.io/lorawan/html/d5/d00/group__lorawan.html): documentation of all classes, member functions and variables generated from Doxygen comments in the source code.

Other useful references:

* [Ns-3 tutorial](https://www.nsnam.org/docs/tutorial/html/ "ns-3 Tutorial"): **Start here if you are new to ns-3!**
* [Ns-3 manual](https://www.nsnam.org/docs/manual/html/ "ns-3 Manual"): Overview of the fundamental tools and abstractions in ns-3.
* The LoRaWAN specification can be downloaded at the [LoRa Alliance website](http://www.lora-alliance.org).
* The Class C timing requirements are defined in LoRaWAN L2 Specification v1.0.4, Section 15.

## Getting help

To discuss and get help on how to use this module, you can open an issue here.

## Contributing

Refer to the [contribution guidelines](.github/CONTRIBUTING.md) for information
about how to contribute to this module.

## Authors

**Upstream module:**
* Davide Magrin
* Martina Capuzzo
* Stefano Romagnolo
* Michele Luvisotto

**Class C extension:**
* Anu Mathew

## License

This software is licensed under the terms of the GNU GPLv2 (the same license
that is used by ns-3). See the LICENSE.md file for more details.

## Acknowledgments and relevant publications

The initial version of this code was developed as part of a master's thesis at
the [University of Padova](https://unipd.it "Unipd homepage"), under the
supervision of Prof. Lorenzo Vangelista, Prof. Michele Zorzi and with the help
of Marco Centenaro.

The Class C extension builds on top of this work and incorporates contributions
from Peggy Anderson and Qiu Yukang.

Publications:

* D. Magrin, M. Capuzzo and A. Zanella, "A Thorough Study of LoRaWAN Performance Under Different
  Parameter Settings," in IEEE Internet of Things Journal. 2019.
  [Link](http://ieeexplore.ieee.org/stamp/stamp.jsp?tp=&arnumber=8863372&isnumber=6702522).
* M. Capuzzo, D. Magrin and A. Zanella, "Confirmed traffic in LoRaWAN: Pitfalls
  and countermeasures," 2018 17th Annual Mediterranean Ad Hoc Networking
  Workshop (Med-Hoc-Net), Capri, 2018. [Link](https://ieeexplore.ieee.org/abstract/document/8407095).
* D. Magrin, M. Centenaro and L. Vangelista, "Performance evaluation of LoRa
  networks in a smart city scenario," 2017 IEEE International Conference On
  Communications (ICC), Paris, 2017. [Link](http://ieeexplore.ieee.org/document/7996384/).
* Network level performances of a LoRa system (Master thesis). [Link](http://tesi.cab.unipd.it/53740/1/dissertation.pdf).
