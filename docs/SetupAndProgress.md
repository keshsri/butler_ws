# ROS Butler Robot Project - Setup and Progress Documentation

## Project Overview

This document outlines the steps taken to set up and develop the ROS workspace for the butler robot project. The goal of this project is to create a robot that can deliver food to tables in a café environment, handling various scenarios as described in the project requirements.

## Initial Setup

### 1. Create ROS Workspace

- A new ROS workspace named `butler_ws` was created using the following commands:
  ```bash
  mkdir -p ~/butler_ws/src
  cd ~/butler_ws/
  colcon build
  source install/setup.bash
  echo "source ~/butler_ws/install/setup.bash" >> ~/.bashrc
  source ~/.bashrc
  ```

### 2. Set Up GitHub Repository

- A GitHub repository named `butler_ws` was created to manage the project files and documentation.
- Initialized a Git repository in the local workspace and connected it to GitHub:
  ```bash
  git init
  git remote add origin https://github.com/keshsri/butler_ws.git
  ```

- Created `.gitignore` and `README.md`:
  ```bash
  echo "install/" >> .gitignore
  echo "build/" >> .gitignore
  echo "log/" >> .gitignore
  echo "# butler_ws" >> README.md
  echo "ROS project for a butler robot delivering food at a café." >> README.md
  ```

### 3. Commit and Push Changes to GitHub

- All files were added, committed, and pushed to the GitHub repository:
  ```bash
  git add .
  git commit -m "Initial commit with workspace setup"
  git push -u origin main
  ```
  ## Custom State Machine Implementation in ROS 2

*End of document.*
