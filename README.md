# Advanced Robotics (INFR112132022) software labs

These instructions are written for ARO labs regarding set up on DICE environment.

The lab instructions are given in the instructions notebook. 
This readme provides you with the instructions for installing the lab requirements.
These instructions are very similar to the [tutorials instructions](https://github.com/ediaro23/tutorials).

## Set up 

### On a DICE machine
On DICE, we will clone the [lab repository](https://github.com/ediaro23/lab) and install the required [dependencies](https://github.com/ediaro23/lab/blob/main/requirements.txt). 
You can "clone" the project to a local folder of your choice.
Open a terminal (CTRL + ALT + T) and follow the commands below:

-   Move to home directory.

```bash
cd ~
```
  
-   Create the aro23 directory if not already done

```bash
mkdir -p aro23 && cd aro23
```

- Clone the lab inside your home directory.

```bash 
git clone https://github.com/ediaro23/lab/
```

- Install dependencies

```bash
python -m pip install --upgrade pip
python -m pip install -r requirements.txt
```    

- You need to update `.bashrc` to include meshcat-server in PATH. Follow the steps below:
    - Open .bashrc for Editing
        ```bash
        nano ~/.bashrc
        ```
    - Add the Following Line to the end of your .bashrc file
        ```bash
        export PATH=$PATH:~/.local/bin
        ```
    - Save and close by pressing CTRL + O to save, followed by CTRL + X to exit.
    - Reload `.bashrc` to apply the changes immediately without restarting the terminal
        ```bash
        source ~/.bashrc
        ```


You should be done! See [below](#using-and-updating-the-notebooks) to check that your installation is working 

### Linux, Python 3, PyPI

On a Linux system with Python 3.8, you can get the dependencies directly with +[pip (see installation procedure and update below)](#installing-pip):
```bash
python3 -m pip install -r requirements.txt
```
NB: you should consider using a [virtualenv](https://docs.python.org/3/library/venv.html)

Once you have the dependencies, you can start the server with `jupyter notebook`

## Using and updating the repository
### Running the instructions notebook
On your terminal, cd into the lab folder:
```bash
cd  ~/aro23/lab/
```
Now run Jupyter notebook with the command
```bash
jupyter notebook .
```
Click on 'instructions.ipynb ' to open the instructions notebook.


### Other helpful instructions
There is a pinocchio cheat sheet available as a pdf. You can also run the notebook "A_pinocchio_cheat_notebook.ipynb" to get a summary of the instructions.
Pinocchio is a bit dense and has its own singular API, it might take some time for you to become familiar with it, but trust me, this will prove largely beneficial.

### Editing the notebook and updates
If the repository changes (for example when the second part of the lab will be pushed / a bug has been found), you will need to update your local
version by "pulling" it from the repository. On a native installation, just go in the folder containing the tutorials and execute ```git pull```


## Side notes

### Installing pip

Pip is a tool for installing and managing Python packages. You can install it with

```bash
sudo apt install python3-pip
```

The default version of +pip installed by +apt is not up to date, so upgrade it with
```bash
python3 -m pip install --upgrade --user
```

In general, running +pip is likely to run an alias on +pip in /usr, so either run it through python3 as explained above, or make sure your path select the right pip executable in your ~/.local. The option --user is kind of optional for recent +pip version, but removing it should work with a warning.

