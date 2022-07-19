# provider_hydrophone
Allows to communicate with hydrophone

![Docker Image CI - Master Branch](https://github.com/sonia-auv/provider_hydrophone/workflows/Docker%20Image%20CI%20-%20Master%20Branch/badge.svg)
![Docker Image CI - Develop Branch](https://github.com/sonia-auv/provider_hydrophone/workflows/Docker%20Image%20CI%20-%20Develop%20Branch/badge.svg?branch=develop)
![GitHub release (latest by date)](https://img.shields.io/github/v/release/sonia-auv/provider_hydrophone)
![Average time to resolve an issue](https://isitmaintained.com/badge/resolution/sonia-auv/provider_hydrophone.svg)


*Please read the instructions and fill in the blanks*


One Paragraph of project description goes here

## Getting Started

Clone current project by using following command :
```bash
    git clone git@github.com:sonia-auv/provider_hydrophone.git
```

These instructions will get you a copy of the project up and running on your local machine for development and testing purposes. See deployment for notes on how to deploy the project on a live system.

**IMPORTANT :** *If you have just imported your repository, please follow the instructions in* [BOOTSTRAP.md](BOOTSTRAP.md) (Once the bootstrap completed, you can remove this comment from the README)

### Prerequisites

First and foremost to run the module you will need to have [docker](https://www.docker.com/get-started?utm_source=google&utm_medium=cpc&utm_campaign=getstarted&utm_content=sitelink&utm_term=getstarted&utm_budget=growth&gclid=CjwKCAjw57b3BRBlEiwA1Imytuv9VRFX5Z0INBaD3JJNSUmadgQh7ZYWTw_r-yFn2S4XjZTsLbNnnBoCPsIQAvD_BwE) installed.

To validate your installation of docker, simply type in

```
docker -v
```

If you receive an output in the likes of :
```
Docker version 19.03.5, build 633a0ea
```

It means you have it installed. If not follow instructions on how to install it for your OS.

## Testing with the Jetson TX2

Run le container
docker run --name provider_hydro -it --privileged -e AUV=LOCAL -e ROS_MASTER_URI=http://192.168.0.104:11311 -e ROS_IP=192.168.0.117 --network=host --mount type=bind,src=/dev,dst=/dev  --mount type=bind,src=/home/sonia/Documents/bags,dst=/home/sonia/ros_sonia_ws/src/bags ghcr.io/sonia-auv/provider_hydrophone/provider_hydrophone:arm64-perception-"branch to test" bash

Test le board
sudo picocom -b 230400 -c /dev/ttyUSB0

## UML

Here is the UML linked to this provider : [Lucidchart UML](https://lucid.app/publicSegments/view/55152d26-6365-45a3-a816-881156453cf4)

## Built With

Add additional project dependencies

* [ROS](http://wiki.ros.org/) - ROS perception framework


## Versioning

We use [SemVer](http://semver.org/) for versioning. For the versions available, see the [tags on this repository](https://github.com/your/project/tags).

## License

This project is licensed under the GNU License - see the [LICENSE](LICENSE) file for details
