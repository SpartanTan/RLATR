# ATR in Docker
Runing different nodes in docker can help isolate each service. Here I create three four Dockerfiles and one docker-compose.ym file to run everything.

## Dockerfiles
So under the workspace/src, there are four folders
```
.
├── factory_db
├── gpss_atr
├── gpss_interfaces
└── rl_atr
```
Each folder contains one Dockerfile. The first layer is built upon gpss_interfaces. FIrst pull the official ros:humble image then install several necessary libraries for this project. One special thing is you need to install 
```
ros-humble-rmw-cyclonedds-cpp
```
Otherwise the communication between containers will not work. (From future: this stuff has to be installed in the rl_atr image as well, because for some reasons it can't find this stuff...Or maybe I just need a source??)
Oh and don't forget to install rviz2 as well.(I actually don't think it's necessary since it should be able to call rviz2 on my host machine, however I didn't success on this, so insatll it is a temporary solution).

The second image is built upon factory_db(Whoever now I think it's better to put it after gpss_atr). You can check the specific Dockerfile for reference, generally speaking not much to do here since it's just simply a configuration package.

The third image is our gpss_atr. The only thing worths notice is that you might need to use --package-select to compile the ```atr_utils``` first, then build the other packages. In my testing, if I throw everything to build together, it might throw errors like "cannot find atr_utils" so that leads to compile failure. 

The last is the rl_atr image. Not many interesting things here(I will try using conda environment in this image, benifits the usage of python libraries like gym, stable-baseline). Oh I have to do this 
```
RUN apt-get install -y ros-humble-rmw-cyclonedds-cpp
```
otherwise it cannot find the ```librmw_cyclonedds_cpp.so```. It is an ugly operation since actually I think it's already installed by the first image. Well, I don't know.

## docker-compose.yml
Great. Now we have all the Dockerfiles we needed, we can now write a docker-compose.yml file to run them alltogether.
Several things important here. 

First, these things should be set for each service.
```
network_mode=host
environment: 
     - "RMW_IMPLEMENTATION=rmw_cyclonedds_cpp"
     - "ROS_DOMAIN_ID=0"
```
For gpss_atr_pipeling, since we will launch the rviz in this environment, the ```environment```, ```volumes``` and ```devices``` should be properly configured so that it can forward GUI to our host machine.
```
environment:
    - "DISPLAY=${DISPLAY}"
    - ...
volumes:
    - /tmp/.X11-unix:/tmp/.X11-unix
    - /dev/dri:/dev/dri
    - ./src/gpss_atr/atr_demo/config:/opt/ros/overlay_ws/install/share/atr_demo/config
devices:
    - /dev/dri:/dev/dri`
```

Yes I know some of them look redundent, I will test which one can be removed in the future. 

In ```volumes```, we mount the atr_demo/config dir into the container, so that if we want to update the parameters in the files under /config, we can update them on our host machine and it will take effect.

OK. I thinks these are everything about setting up this project in Docker. I will provide the command below for using it.

```
# 1. change directory to ros_ws
# 2. build all the docker images
docker compose build
# 3. Run the docker compose 
docker compose run
```

### Other stuff
If there's nothing changed in the factory_db, gpss_atr and gpss_interfaces, you can uncomment the `build ` item in the docker-compose.yml to simply use the pre-built image. This will make docker compose build much faster.

But do remember to build everything in the first time. All the images are built and stored locally.