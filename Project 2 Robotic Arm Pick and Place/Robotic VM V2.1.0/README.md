To follow along with the exercises in these lessons, you will need to be running Ubuntu Linux with ROS installed. So that everyone can follow along in the same environment we are providing you with a virtual machine (VM) that is all setup with everything you need.

It may also be a good idea to keep a backup of this VM image to avoid downloading again if you need it. But just in case you do not do that there is a backup snapshot built into the VM that will allow you to restore it to its original state at anytime!

To extract the compressed image in Windows:
1. Download the appropriate version of 7-zip from [here](http://www.7-zip.org/download.html)
2. Right-click and select "7-zip" > "Extract Here"

To extract at the command line in macOS or Linux:
1. Open Terminal and navigate to directory containing the compressed image
2. execute `$ unzip robond_vm.zip`

In the zipped archive in addition to the VM disk image, there are two other small configuration files that you may or may not need.  

We recommend using [VMWare](http://www.vmware.com/) to run the VM.  VMWare provides the [Workstation Player](http://www.vmware.com/products/player/playerpro-evaluation.html) free for Windows and Linux operating systems.  

We are actively pursuing an agreement with VMWare to offer free licenses to students in this program for their [VMWare Fusion Player for Mac](http://www.vmware.com/products/fusion.html), but for now it only comes with a brief free trial period.  To use the free Workstation player for Windows on a Mac, you can use BootCamp (a built in Mac tool) to create a Windows partition on your Mac and download the player for WIndows.  Alternatively, you can check out a free trial of VMWare Fusion and consider buying it.  

Whichever operating system you're on, download the appropriate player and follow these OS specific instructions:

#### MacOS


1. Download and install VMPlayer
2. Download the image from supplied link
3. Unzip the image 
4. Open VMware Application
5. Click File then New
6. Select Import an existing virtual machine
7. Click Choose File... and navigate to your unzipped image
8. Click save when prompted for virtual machine name 
9. **If** an error pops up click Retry
10. From here you should be launched into your new shiny virtual machine!

#### Linux

1. Download and install VMPlayer
2. Download the image from supplied link
3. Unzip the image 
4. OpenVM Player
5. Select File -> Open a Virtual Machine 
6. Navigate to the directory of the unzipped image and select the image. 
7. If the player fails to open the image perform steps 5 and 6 one more time.
8. Select the imported image on the left side of the player
9. Click run
10. From here you should be launched into your new shiny virtual machine!

#### Windows

1. Download and install VMPlayer
2. Download the image from supplied link
3. Unzip the image 
4. Open VMware
5. Press "Open a Virtual Machine"
6. Navigate to the folder that contains the VM files
7. Select the larger **.ova** file
8. Wait file to be imported (May take some time)
9. **If** an error pops up click Retry
10. From here you should be launched into your new shiny virtual machine!

Before getting your VM up and running, you will need to make sure that you allocate enough resources to allow it to operate efficiently. With your VM shutdown, navigate to the VM settings under the menu for your VM. Look for a section labeled "Processors and Memory"; this is where you will change the number of cores and amount of RAM that you allocate from your own machine to the VM. If you have the resources on your machine, we recommend allocating at least 2 processor cores and 4 GB of RAM. If you are able to allocate more resources, feel free to do so. The more you allocate the better performance you will get! You're now all set to boot up your VM!

Upon your first boot you will be prompted to choose a keyboard layout of your choice. Once you select a keyboard you will not be prompted for this again. If you would like to change your option you can reset this feature by entering `udacity_reset` in a terminal of your choice and restarting the VM.

Once you are up and running, you might be asked to input a password to enter the VM.  **The password for the VM is `robo-nd`**

To open a terminal in your VM press`ctrl-Alt-t` (or `ctrl-option-t` on a Mac).  You should get a terminal window that looks like this: