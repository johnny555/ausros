# Example project for using LLM's to code 

Look in the `docs` folder to see some example prompts. Feel free to modify these as you go! 

# Structure 

`src` folder includes the turtlebot3 simulation software, plus rqt_robot_steering with TwistStamped option.

Note we are using jazzy (not humble) because gazebo (harmonic) binaries exist for arm64, but not on humble. 

# Requirements 

This should work on Windows, Mac or Linux. You just need VS Code and docker. It stands up a VNC server in the container that is accessible from localhost:6080

# Steps 

## Setup 
Clone this repo using: 

`git clone --recurse-submodules https://github.com/johnny555/ausros.git`

`cd ausros` 

Install VS Code and Docker Desktop. 

Open `ausros` folder within VS Code, hit `Ctl+Shift+P` "Dev Container: Open Folder In Container" and open the `ausros` folder (not the `.devcontainer` folder! ).

Install the GitHub co-pilot VS Code extension and sign up for it. 

## Test it works 

Inside the dev container, run 

`colcon build --merge-install --symlink-install`

Set the robot model to waffle
`export TURTLEBOT3_MODEL=waffle`

Set the display output to `:1`

`export DISPLAY=:1`

then launch a simulation
`source install/setup.bash`
`ros2 launch turtlebot3_gazebo turtlebot_world.launch.py` 

in a seperate terminal, export display
`export DISPLAY=:1`
and run the teleop control 
`source install/setup.bash`
`ros2 run rqt_robot_steering rqt_robot_steering` 

## Teleop 

Go to `http://localhost:6080`

press connect, you should now see gazebo and a rqt_robot_steering window. 

Ensure the "stamped" check box is checked and now drive the robot. 

## First steps with AI 

See if you can get the AI to write a ros2 launch file to automate the above steps to get teleop working. 

Don't forget to reference `docs/architecture.md`. 

## Plan 

Pass the challenge-task.md to the chat, use # to reference it. Use a thinking model (i.e. Claude Sonnet 3.7 Thinking) and ask it to break the project down into small steps (use the Ask type). 

Overwrite `project-overview.md` with your plan. Adjust it to your liking. 

## Step through it 

Slowly walk through your plan, make sure to test it works in simulation. 

Don't be afraid to ask your LLM for help along the way. 