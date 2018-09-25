# uav-dronekit-scripts
This project is to work with a docker container to perform flight scenarios, in a stable, ready to deploy environment using the Docker Image from the `uav-sitl-docker` [repo](https://github.com/emantas/uav-sitl-docker).

## Download Docker Image
```
docker pull evans000/uav-sitl
```

# Docker Start Up
```
docker run -it --privileged \
 --env=LOCAL_USER_ID="$(id -u)" \
 -v /tmp/.X11-unix:/tmp/.X11-unix:ro \
 -v ~/uav-dronekit-scripts:/uav-dronekit-scripts:rw \
 -e DISPLAY=:0 \
 -p 14556:14556/udp \
 -p 8080:8080 \
 dronekit-sim bash
```

### Spawn a new Docker Container
```
docker exec -it <container-id> bash
```

#### Important
Docker may not run if not called with sudo. In this case add `sudo` before the execution of the docker command

# Dronekit-sitl
Starting the dronekit sitl , custom made flight missions can be run on top of it. The flight script will be connected on `127.0.0.1:14551`, Mavproxy will be connected on `127.0.0.1:5760` and output a connection on your IP ex. `192.168.2.10:14550`
### Docker Terminal 1
```
dronekit-sitl copter --home=-9.430663,38.893766,0,180
```

### Docker Terminal 2
Replace the `dronekit-script.py` with the flight mission file name.
```
cd dronekit_scripts
python dronekit-script.py --connect 127.0.0.1:14551
```

### Docker Terminal 3
Replace the the last `--out` flag with the IP of your machine (not the Docker internal IP)
```
mavproxy.py --master tcp:127.0.0.1:5760 --out udp:127.0.0.1:14551 --out xxx.xxx.xxx.xxx:14550
```

## QGroundStation
On your Computer open `QGroundStation` and create a `UDP` connection on port `14551`
