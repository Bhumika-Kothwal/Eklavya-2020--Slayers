<!-- SETUP INSTRUCTUIONS -->
## Setup Instructions
* Installing make  
  * Run update command to update package repositories and get latest package information.
  ```sh
  $ sudo apt-get update -y
  ```
  * installing make using the ```apt-get``` command
  ```sh
  $ sudo apt-get -y install make
  ```
  * You can verify if it is correctly installed by checking its version and the path where it is stored
  ```sh
  $ make --version
  GNU Make 4.1
   
  $ which make
  /usr/bin/make
  ``` 

* Installing gcc compiler for C file compilation 

  * Start by updating the packages list
  ```sh
  $ sudo apt update
  ```
  * Install the ```build-essential``` package by typing
  ```sh
  $ sudo apt install build-essential
  ```
  * Install the manual pages about using GNU/Linux for development
  ```sh
  $ sudo apt-get install manpages-dev
  ```
  * To validate that the GCC compiler is successfully installed, use the ```gcc --version``` command which prints the GCC version
  ```sh
  $ gcc --version
  ```
  
    
* Setup of Coppelia-Sim Software
  * Download Coppelia-Sim Edu [here](https://www.coppeliarobotics.com/downloads) by choosing the correct platform.
  * If you are a Ubuntu user, follow the below steps for further set-up:  
      
    1.Extract the downloaded zip file and move the file to Home directory.  
      
    2.Open the terminal in Home directory and type the command below-
    ```sh
    $ gedit .bashrc
    ```
    3.Now, add the below lines at the end of the file-
    ```sh
    alias coppelia-sim='$HOME/CoppeliaSim_Edu_V4_0_0_Ubuntu16_04/coppeliaSim.sh'
    ```
    4.Type the below command in your terminal to open the CoppeliaSim environment-
    ```
    $ coppelia-sim
    ```
