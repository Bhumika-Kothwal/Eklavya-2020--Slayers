# Labyrinth Simulation - CoppeliaSim
Line following Maze solving bot with shortest path in CoppeliaSim environment

<!-- TABLE OF CONTENTS -->
## Table of Contents

* [About the Project](#about-the-project)
  * [Tech Stack](#tech-stack)
  * [File Structure](#file-structure)
* [Getting Started](#getting-started)
  * [Prerequisites](#prerequisites)
  * [Installation](#installation)
* [Usage](#usage)
* [Results](#results)
* [Future Work](#future-work)
* [Contributors](#contributors)
* [Acknowledgements and Resources](#acknowledgements-and-resources)
* [License](#license)


<!-- ABOUT THE PROJECT -->
## About The Project
[**maze solving screenshot**](https://github.com/Bhumika-Kothwal/Eklavya-2020--Slayers/tree/master/doc/results/result%20screenshots)  

* **Aim**   
Making a line following maze solving bot with shortest path algorithm in CoppeliaSim in C-language. 
* **Description of project**  
We have made a bot which will be capable enough of exploring an unknown path on its own without any user interaction during the exploration.
Our motive was to make a bot that can explore the maze and find the shortest path in the minimum time possible by traveling the minimum distance.
Hence our project requires use of algorithms and data structures as the major part. We have used the Legacy Based Remote API for server side so that the written code is in C language.  

Refer this [documentation](https://github.com/Bhumika-Kothwal/Eklavya-2020--Slayers/blob/master/doc/report.pdf)

### Tech Stack
This section lists the technologies you used for this project.
* [Coppeliasim](https://www.coppeliarobotics.com/)  

### File Structure
    .
    ├── docs                    # Documentation files
    │   ├── report.pdf          # Project report
    │   └── results             # Folder containing screenshots, videos of results
    ├── dry_run                 # Source files and vrep scenes for dry run of bot
    │   ├── src                 # Source files and code for exploration of maze
    │   ├── vrep_scenes         # Folder containing scene with bot and path
    │   └── Makefile            # for producing final executable file
    ├── final_run               # Source files and vrep scenes for final run of bot
    │   ├── src                 # Source files and code for final of maze
    │   ├── vrep_scenes         # Folder containing scene with bot and path
    │   └── Makefile            # for producing final executable file
    ├── LICENSE
    ├── README.md 
    └── SETUP.md                
        

<!-- GETTING STARTED -->
## Getting Started

### Prerequisites

* See [SETUP.md](https://github.com/Bhumika-Kothwal/Eklavya-2020--Slayers/blob/master/SETUP.md) for the installation steps.
* List of softwares with version tested on 
```sh
$ gcc --version
gcc (Ubuntu 5.4.0-6ubuntu1~16.04.12) 5.4.0

$ make --version
GNU Make 4.1
```


### Installation
1. Clone the repo
```sh
git clone https://github.com/your_username_/Project-Name.git
```


<!-- USAGE EXAMPLES -->
## Usage
*  **For Dry Run**  
   1. Open the folder [dry_run](https://github.com/Bhumika-Kothwal/Eklavya-2020--Slayers/tree/master/dry_run)
   2. Run the make command in the terminal as given below. (This should lead to formation of two folders-bin, obj)
   ```sh
   $  make clean
   $  make
   ```
   3. Open the CoppeliaSim environment by running the command below in the terminal-
   ```sh
   $  coppelia-sim
   ```
   4. Open the desired scene and start the simulation.
* **For Final Run**
  1.  Open the folder [final_run](https://github.com/Bhumika-Kothwal/Eklavya-2020--Slayers/tree/master/final_run) 
  2.  Follow the same steps 2-4 as stated for Dry Run. 


<!-- RESULTS AND DEMO -->
## Results

[**result screenshots**](https://github.com/Bhumika-Kothwal/Eklavya-2020--Slayers/tree/master/doc/results/result%20screenshots)  
[**result video**](https://github.com/Bhumika-Kothwal/Eklavya-2020--Slayers/tree/master/doc/results/result%20video)  


<!-- FUTURE WORK -->
## Future Work
- [ ] Improving the efficiency of back-tracking
- [ ] Introducing wheel encoders in bot


<!-- CONTRIBUTORS -->
## Contributors
* [Bhumika-Kothwal](https://github.com/Bhumika-Kothwal)
* [pratamjain](https://github.com/pratamjain)


<!-- ACKNOWLEDGEMENTS AND REFERENCES -->
## Acknowledgements and Resources
* [SRA VJTI](http://sra.vjti.info/) Eklavya 2020  
* Refered [this](https://www.youtube.com/watch?v=PwGY8PxQOXY&list=PLjzuoBhdtaXOoqkJUqhYQletLLnJP8vjZ) for understanding and working with CoppeliaSim and for model of bot
* Refered [this](https://www.coppeliarobotics.com/helpFiles/) for implementing Legacy Based Remote API and Remote API functions for C language.Below flowchart will help you access the required information from the page cited above.   
```sh
    CoppeliaSim User Manual     
    ├── ...           
    ├── Writing code in and around CoppeliaSim                  
    │   ├── ...     
    │   └── CoppeliaSim API framework     
    │       ├── ...       
    │       ├── Remote API        
    │       │   ├── ...       
    │       │   └── Legacy remote API                               #Using Legacy Remote API        
    │       │       ├── Enabling the remote API - client side       #Information about the client side requirements       
    │       │       ├── Enabling the remote API - server side       #Information about the server side requirements       
    │       │       ├── ...       
    │       │       ├── Remote API functions(C/C++)                 #Information about the functions to be used, their purposes, prototypes, return types       
    │       │       └── ...       
    │       └── ...       
    └── ...       
```


<!-- LICENSE -->
## License
Details of license can be found [here](LICENSE). 
