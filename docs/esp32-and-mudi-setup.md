# ESP32 and Mudi 7 setup

Links Mission Planner to the Pixhawk: TELEM1, then ESP32 running DroneBridge, then Mudi 7 WiFi, then Tailscale for remote access. **Not yet tried on this hardware.** Steps come from the DroneBridge and GL.iNet docs, linked below. They were read on 2026-11-04, when the current DroneBridge release was v2.4.1.

DroneBridge warns that it "may crash, reset, freeze or stop operating abruptly" and that it must not be relied on for safe operation. This link is for monitoring and missions. It is not a safety stop. Keep the radio kill and stay within reach of the key.

## 1. Flash DroneBridge

1. Connect the ESP32 by USB.
2. Use the [official online flasher](https://drone-bridge.com/flasher/), which DroneBridge recommends. Its [install page](https://dronebridge.gitbook.io/docs/dronebridge-for-esp32/installation) also covers `esptool`. If you use `esptool`, take the flash addresses from the release you downloaded, since they differ between DroneBridge's own documents.

Choose the plain ESP32-S3 firmware for this board. DroneBridge lists the S3 as supported, but its top recommendation is the C6, and the S3 terminal board is not one of its official boards, so expect less support.

## 2. Wire to TELEM1

Power the ESP32 from one source only. DroneBridge notes that some modules don't accept 5 V input and USB at the same time, so unplug USB before using TELEM1 power, and check your board's rules.

| Pixhawk TELEM1 | ESP32 |
|---|---|
| Pin 1, VCC (5 V) | 5 V |
| Pin 2, TX | GPIO41, set as **UART RX** |
| Pin 3, RX | GPIO42, set as **UART TX** |
| Pin 6, GND | GND |

- Pixhawk TX goes to ESP32 RX, and the reverse. Both UARTs are 3.3 V logic, per the [DroneBridge wiring page](https://dronebridge.gitbook.io/docs/dronebridge-for-esp32/hardware-and-wiring).
- That page's XIAO wiring bullets read as straight-through (Pixhawk TX to the pin "configured as TX"). This conflicts with its own config page, which defines the setting as the ESP32's RX or TX GPIO, and with normal UART wiring. Follow the table above.
- GPIO41 and GPIO42 are the choice in `docs/wiring-and-code-map.md`, which also lists GPIO 0, 3, 19, 20, and 43-46 to avoid. DroneBridge says not to use a board's pins labelled TX and RX, because they carry the debug console, nor strapping pins. Go by the GPIO number. Board labels can differ.
- The module appears to have a U.FL connector, so fit an antenna and route it outside the steel bay.
- Leave RTS and CTS unconnected, and set both pins to 0 in the web interface. `BRD_SER1_RTSCTS` is already 0 in `ardupilot/mower.param`.

## 3. Prepare the Mudi

1. Log in to the admin panel, by default at `http://192.168.8.1`.
2. Under Wireless, use a Main or IoT network with a 2.4 GHz band enabled. Note its SSID and password. DroneBridge's mode table lists 802.11b rates for client mode, which are 2.4 GHz only.
3. Not confirmed by either vendor's docs for this pairing: whether the ESP32 joins a WPA3-only or MLO network. Start with plain WPA2 on 2.4 GHz.

## 4. Configure DroneBridge in client mode

1. Join the ESP32's WiFi, `DroneBridge for ESP32`, password `dronebridge`. The README spells the name `DroneBridge ESP32`, so go by what you see.
2. Open `http://192.168.2.1` or `http://dronebridge.local`. You may need to turn off cellular on a phone.
3. [Set](https://dronebridge.gitbook.io/docs/dronebridge-for-esp32/configuration):
   - **Mode:** WiFi Client Mode
   - **SSID, Password:** the Mudi's network
   - **UART TX Pin:** 42, **UART RX Pin:** 41
   - **UART RTS Pin, UART CTS Pin:** 0
   - **UART baud:** 115200
   - **UART serial protocol:** MAVLink
4. Hit **save & reboot**. The ESP32 then joins the Mudi.
5. In the Mudi's **Clients** page, find the ESP32's IP and click [Reserved IP](https://docs.gl-inet.com/router/en/4/interface_guide/clients/) (firmware v4.8 or later), so the address stays fixed.

If you get locked out, a short press of the BOOT button returns the mode and WiFi settings to access point mode. A press over 1.8 s resets everything.

## 5. Set up the Pixhawk

`ardupilot/mower.param` already sets these. They match `SERIAL1_BAUD` 115 (115200) and MAVLink2:

```text
SERIAL1_PROTOCOL,2
SERIAL1_BAUD,115
BRD_SER1_RTSCTS,0
```

## 6. Connect Mission Planner on the Mudi's WiFi

Use the ESP32's IP from step 4, with either:

- **TCP**, port **5760**.
- **UDPCI**, port **14550**. [DroneBridge says](https://dronebridge.gitbook.io/docs/dronebridge-for-esp32/configuration/missionplanner-setup) to use UDPCI in client mode, not plain UDP, because the ESP32 only sends UDP to a client after that client has sent it a packet.

Mission Planner may list the ESP32 as its own MAVLink device. Choose the mower in the drop-down. DroneBridge warns that changing ESP32 settings from Mission Planner applies them immediately and reboots the ESP32, which can lose the link for good if the settings stop matching.

## 7. Remote access with Tailscale

GL.iNet lists the [GL-E5800 (Mudi 7) as supported](https://docs.gl-inet.com/router/en/4/interface_guide/tailscale/) and calls the feature beta.

1. Create a Tailscale account, and add your phone or laptop to it.
2. In the Mudi's admin panel: **APPLICATIONS > Tailscale**, enable, **Apply**.
3. Click the **Device Bind Link** when it appears, log in, and click **Connect**.
4. Enable **Advertise LAN Subnets** (named Allow Remote Access LAN before firmware v4.9), and **Apply**.
5. In the Tailscale admin console, open the Mudi's three-dot menu, then **Edit route settings**, and approve the subnet route.
6. From a remote device on the same Tailscale network, connect Mission Planner to the ESP32's LAN IP as in step 6.

GL.iNet advises against running Tailscale alongside the OpenVPN client, WireGuard client, GoodCloud Site to Site, ZeroTier, or AstroWarp, because routes can conflict.

**Not confirmed by these docs:** that TCP 5760 and UDPCI work through the subnet route, or how much latency cellular adds. Test both before relying on them.

## Link loss and `FS_GCS_ENABLE`

`FS_GCS_ENABLE` is 1 and `FS_ACTION` is 5, so once Mission Planner has connected, losing its heartbeat for `FS_GCS_TIMEOUT` (default 5 s) plus `FS_TIMEOUT` (default 1.5 s) disarms the mower. Closing Mission Planner, an ESP32 reset, or a cellular dropout will all do this. If it trips too often, raise the timeout. Don't turn it off until you have decided what should stop the mower when nobody can see it.
