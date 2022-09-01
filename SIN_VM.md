# Sin Virtual Machine

## Si tenes Ubuntu 20
```sh
sudo apt update
```

```sh
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
```

```sh
curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
```

```sh
sudo apt update && sudo apt install -y ros-noetic-desktop-full 
```

```sh
sudo apt install git
```

```sh
cd ~
git clone https://github.com/ekumenlabs/ros_workshop.git
```


## Si tenes Ubuntu != 20

Install docker 

```sh
 sudo apt-get update
 sudo apt-get install \
    ca-certificates \
    curl \
    gnupg \
    lsb-release
```

```sh
sudo mkdir -p /etc/apt/keyrings
 curl -fsSL https://download.docker.com/linux/ubuntu/gpg | sudo gpg --dearmor -o /etc/apt/keyrings/docker.gpg
```

```sh
 echo \
  "deb [arch=$(dpkg --print-architecture) signed-by=/etc/apt/keyrings/docker.gpg] https://download.docker.com/linux/ubuntu \
  $(lsb_release -cs) stable" | sudo tee /etc/apt/sources.list.d/docker.list > /dev/null
```

```sh
 sudo apt-get update
 sudo apt-get install docker-ce docker-ce-cli containerd.io docker-compose-plugin
```

Setup docker

```sh
sudo groupadd docker
```
```sh
sudo usermod -aG docker $USER
```
```sh
newgrp docker
```

Install git
```sh
sudo apt install git
```

Clonar repositorio de ekumen
```sh
cd ~
git clone https://github.com/ekumenlabs/ros_workshop.git
```

Buildear container
```sh
cd ros_workshop
./docker/build.sh
```

Run container
```sh
cd ros_workshop
./docker/build.sh
```


TODO: Continuar con trainer!



