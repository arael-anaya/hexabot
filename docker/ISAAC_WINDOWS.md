# Isaac Sim on Windows + ROS 2 container

Isaac Sim runs natively on Windows (RTX renderer needs real GPU access, which
WSL2 containers don't have). The ROS 2 Humble container stays in Docker.

## 1. Install Isaac Sim
1. Download the Windows package from the Isaac Sim download page (Latest Release).
2. Unzip to `C:\isaacsim` (short path, no spaces).
3. Update the NVIDIA driver (Game Ready/Studio, current branch).

## 2. Install the streaming client
Download "Isaac Sim WebRTC Streaming Client" (same download page), install it.
Server `127.0.0.1`, signal port `49100`, stream port `47998`.

## 3. Start Isaac Sim (PowerShell)
```powershell
cd C:\isaacsim
$env:ROS_DOMAIN_ID = "0"
$env:RMW_IMPLEMENTATION = "rmw_fastrtps_cpp"
$env:PATH = "$env:PATH;C:\isaacsim\exts\isaacsim.ros2.bridge\humble\lib"
.\isaac-sim.streaming.bat
```
Wait for "Isaac Sim Full Streaming App is loaded" (first run takes minutes),
then click Connect in the client. First run may need `ACCEPT_EULA` prompts.

## 4. Let ROS in Docker see Isaac Sim (DDS discovery)
Docker's NAT blocks DDS multicast, so use WSL mirrored networking + host mode.
1. Create/edit `%UserProfile%\.wslconfig`:
   ```ini
   [wsl2]
   networkingMode=mirrored
   ```
2. `wsl --shutdown`, then restart Docker Desktop.
3. Windows Firewall: allow inbound UDP for Isaac Sim (`kit.exe`) and the
   streaming client, or allow UDP 7400-7600 on private networks.
4. Start ROS: `docker compose -f docker/compose.yaml up -d --build`
   (`network_mode: host`; no hexabot_net needed anymore).
5. Test: in Isaac Sim enable a ROS 2 bridge publisher (e.g. clock), then in
   the container: `ros2 topic list` and `ros2 topic echo /clock`.

If topics don't appear, the fallback is Fast DDS unicast peers, or running ROS
directly in a WSL2 Ubuntu distro (also mirrored).

## Requirements checklist
- NVIDIA RTX GPU with NVENC, 8 GB VRAM is the bare minimum
- Windows 10/11, current NVIDIA driver
- WSL >= 2.0.0 (`wsl --update`) for mirrored networking
- Same `ROS_DOMAIN_ID=0` and `rmw_fastrtps_cpp` on both sides
- Open: TCP 49100, UDP 47998 (streaming), UDP DDS ports
