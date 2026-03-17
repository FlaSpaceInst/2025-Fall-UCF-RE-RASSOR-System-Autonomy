# Setup Guide for Ubuntu 20.04 LTS on a Board with ROS2 Integration
---
## 1. Board Selection and Preliminary Requirements
---
Locate what Board you will be building this on, current software requirements limit this to boards that can run Ubuntu 20.04 LTS, this board will also need a wifi connection or wifi adaptor in order to work properly

## 2. Ubuntu and SSH Configuration
---
This section will detail flashing Ubuntu onto the board and configuring ssh to make changes to the board's files. This needs to happen before anything can be done.

#### Installing Ubuntu
---
Flash Ubuntu 20.04 LTS onto the board.

Use this flasher: https://etcher.balena.io/

Use this link for Ubuntu if using Le Potato : https://distro.libre.computer/ci/ubuntu/20.04/

*Note: the Le Potato's Ubuntu version will have these credentials:*

*Username: `root`*

*Password: `root`*

#### Installing and Configuring SSH
---
##### Install nano:

```bash
sudo apt-get install nano
```

##### Install openssh-server:

```bash
sudo apt-get update
sudo apt-get install openssh-server
```

##### Edit `sshd_config` to allow root login:

*Run the following commands:*

```bash
cd /etc/ssh/
nano sshd_config
```

Within the text editor, navigate to the Authentication section and delete the `#` in front of `PermaRootLogin` and change what comes after it to `yes`. 

**Nano Editor Quick Guide**:
- **Arrow Keys:** Use the arrow keys to move the cursor around the text file. 
- **Editing:** Just start typing to insert text at the cursor position. 
- **Save Changes:** Press `CTRL` + `O`, then press `Enter` to confirm the filename. 
- **Exit:** Press `CTRL` + `X`. If you have unsaved changes, Nano will ask if you want to save them.

##### Enable LLMNR and MulticastDNS:

*Run the following commands:*

```bash
cd /etc/systemd/
nano resolved.conf
```

Within the text editor, delete the `#` in front of both `LLMNR` and `MulticastDNS` and change them to `yes`.
##### Change your hostname:

*Run the following commands:*

```bash
cd /etc/
nano hostname
```

Edit the hostname to a desired name and reboot. Now you should be able to `ssh` into `root@yourhostname`.

*Note: A reboot can be done using `reboot`.*

*From here, you can set up a user or just use root to set up the software. In this guide we will use root. If you create your own user, the only difference will be file paths and using sudo.*
   
## 3. Software Installation and Environment Setup
---
Once you are successfully ssh'd into the board, you need to set up it's environment. This section covers installing and validating ROS2.

#### Installing and Setting Up ROS2:
---

Set up the Ros2 Debian Package through these instructions [LINK](https://docs.ros.org/en/foxy/Installation/Ubuntu-Install-Debians.html). This will have you install both `python3` and `Ros2 Foxy`.

Once you reach the source step at the end, you will need to add a line to the end of the `.bashrc` file:

*Run the line:*

```bash
cd ~
nano .bashrc
```

*Then, add this line of code to the very end of the .bashrc file.*

```bash
source /opt/ros2/foxy/setup.bash
```

This allows ubuntu to automatically source ros2 on start up so the user does not have to.

#### Validating ROS2 Installation:
---

To verify if the setup was successful, type `ros2` into the terminal and it should print out all the commands you can run with ros2 foxy.

#### Install Additional Dependencies
--- 
##### Install pyserial:

```bash
pip install pyserial
```
##### Install flask:

```bash
pip install flask
```


## 4. Cloning GitHub Repositories
---
This section will go over the cloning and configuring of the repositories as well as the installation of additional software.
##### Install Git:

```bash
sudo apt-get install git
```

#### Generate a new SSH Key for GitHub use:
---
Run this line in your terminal. Use the email associated with your GitHub account instead of the example email:

```bash
ssh-keygen -t rsa -b 4096 -C "your_email@example.com"
```

This should save the key in the path `~/.ssh/rsa.pub`.

#### Add the SSH key to GitHub
---
To use the generated SSH key and clone repositories, you will need to link the key to your GitHub account.

- Go to GitHub and log into your account.
- Click on your profile photo (top right) > **Settings**.
- In the left sidebar, click on **SSH and GPG keys**.
- Click **New SSH key**.
- Provide a title for the key (e.g., "Ubuntu machine" or any descriptive name).
- In the "Key" field, paste your key.
- Click **Add SSH key**.

Now, test the SSH connection with the following line:

```bash
ssh -T git@github.com
```

The first time you do this, you'll receive a message about verifying the GitHub server. Type `yes` and press `Enter`.

If everything is set up correctly, you should receive a message like "Hi \[your username]! You've successfully authenticated...". Now, you should be able to clone repositories onto the board. 

*Note: Remember to use the SSH URL when cloning.*
   
## 5. Repository Setup and Workspace Configuration
---
This section details the cloning and configuring of the repositories.

#### Setting Up the Workspaces
---
Create the directories `software_ws` and `hardware_ws` in the root directory:

```bash
cd /root/
mkdir software_ws
mkdir hardware_ws
```

#### Cloning the Repositories
---
Go into the `software_ws` folder and clone the `ezrassor_controller_server` repository into it:

```bash
cd /root/software_ws/
git clone git@github.com:FlaSpaceInst/ezrassor_controller_server.git
```

Go into the `hardware_ws` folder and clone the `rassor_serial_forward` repository into it:

```bash
cd /root/hardware_ws/
git clone git@github.com:FlaSpaceInst/2023-ucf-L14---RE-RASSOR-Autonomy-for-Mark-2-Computing.git
```

#### Workspace Cleanup
---
Once the repositories are cloned, you need to run a few commands to remove files that are not needed:

```bash
cd 2023-ucf-L14---RE-RASSOR-Autonomy-for-Mark-2-Computing
rm -rf StepperTesting
rm -rf arduino_client
mv Image\ Creation\ Files/ ~/ImageCreatingFiles #moves to home dir
mv rassor_serial_forward ..` #moves to hardware_ws
```

## 6. Setting up the ROS2 Nodes
---
Now that the workspaces are cleaned up, we will build the ROS2 Nodes and set up their automation.

#### Building the Nodes:
---
You will need to navigate to the respective folders and run build commands for each.
##### Build hardware_ws:

```bash
cd ~/hardware_ws
colcon build
```

##### Build software_ws:

```bash
cd ~/software_ws
colcon build
```

#### Setting up Automation:
---
Change directories into the working user folder and move the `systemctl` start files into the home directory.

*Inside the git repo (cd hardware_ws/2023....):*

```bash
mv Systemctl\Starter\files/* ~/
```

*Also inside the git repo (cd hardware_ws/2023....):*

```bash
mv Systemctl\Starter\files/01-arduino.conf  ~/etc/udev/rules.d
```


Now you will need to run commands to start the services:


*Reload the systemd Configuration:*
```bash
systemctl daemon-reload
```

*These lines allow read, write, and execute permissions to everyone:*
```bash
chmod 777 /etc/systemd/user/ros2_start.sh
chmod 777 /etc/systemd/system/ros2-start-forwarder.service
chmod 777 /etc/systemd/system/ros2-start-processes.service
chmod 777 /etc/systemd/user/ros2_start_forwarder.sh
```

*Enable and check the ROS Forwarder Service:*
```bash
systemctl enable ros2-start-forwarder.service
systemctl status ros2-start-forwarder.service
```

*Enable and check the ROS Processes Service:*
```bash
systemctl enable ros2-start-processes.service
systemctl status ros2-start-processes.service
```
