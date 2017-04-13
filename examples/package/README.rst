Packaging Secure ROS
====================

This folder contains tools (vagrant and ansible) for creating Ubuntu debian packages for Secure ROS. Please see the section on :ref:`vagrant_and_ansible` for details on installing and using vagrant and ansible. 

Supported architectures and packages
------------------------------------

The following architectures are supported. 

* amd64 (64-bit PC)
* i386 (32-bit PC)

The following releases are supported

* ROS `indigo` (Ubuntu 14.04) on the `indigo-devel` branch.
* ROS `kinetic` (Ubuntu 16.04) on the `kinetic-devel` branch.

The ``secure-ros-<release>-ros-comm`` package is created for each of the architectures. This includes modified versions of libraries (e.g ``libroscpp.so``) and python modules (``roslaunch``, ``rospy``) which will be installed in ``/opt/secure_ros/<release>/``. 

Generating the packages
-----------------------
The packages can be generated as follows. ::

  vagrant up 

The package is created in the root folder of the repository: ``deb/<arch>/secure-ros-<release>-ros-comm_<version>_<arch>.deb``. The build is preserved if the VMs are halted, but will need to be rebuilt if the VMs are destroyed.

Vagrant tasks 
~~~~~~~~~~~~~

- Bring up the VMs as specified in the ``secure_ros.yaml`` configuration file.
- Provision the VMs as specified in the ``secure_ros.yaml`` files (build, create and install the Secure ROS package).

Ansible playbooks
~~~~~~~~~~~~~~~~~

- ``plays/install-secure-ros.yaml``: Please see :ref:`pubsub_ansible`.
- ``plays/build-secure-ros.yaml``: Please see :ref:`pubsub_ansible`.


.. _vagrant_and_ansible:

Vagrant and ansible
===================

We use *ansible* and *vagrant* to create and configure multiple VMs to run the examples and create debian package for Secure ROS. 

Install Vagrant and ansible
---------------------------

We support Ubuntu (14.04, 16.04) and OS X. It is relatively easy to install on Ubuntu, while on Mac OS X, it is a bit more involved.

Ubuntu
~~~~~~

Install virtualbox::

  sudo apt-get install -y virtualbox

Install vagrant::

  wget https://releases.hashicorp.com/vagrant/1.8.5/vagrant_1.8.5_x86_64.deb
  sudo dpkg -i vagrant_1.8.5_x86_64.deb

Install ansible::

  sudo apt-add-repository ppa:ansible/ansible -y
  sudo apt-get update
  sudo apt-get install ansible -y

OS X 
~~~~

Install virtualbox from https://www.virtualbox.org/wiki/Downloads.

Install vagrant from https://releases.hashicorp.com/vagrant. 

Install ansible (this is a bit more involved. The instructions from https://valdhaus.co/writings/ansible-mac-osx/ are provided here for convenience. If you already have Xcode and ``pip`` installed, you can skip those respective steps.

- Install Xcode
- Install ``pip`` ::

    sudo easy_install pip

- Install ansible using ``pip`` ::

    sudo pip install ansible --quiet

Start up vagrant 
----------------

Vagrant may be used to set up the VMs using two steps.  These commands must be called from the example or package folders that contains ``Vagrantfile``.

To fire up and configure the VMs using vagrant ::

  vagrant up 

Trouble-shooting
----------------

The most common issues are as follows.

- Make sure you have an internet connection. 
- Make sure that you have as many cores and RAM available as needed in the example. 
- There might be a bug in ansible that may cause it to fail. You may try again as follows. ::

    vagrant provision

- If it fails at the same spot, you may restart the VMs and try again. ::

    vagrant halt 
    vagrant up --no-provision 
    vagrant provision


