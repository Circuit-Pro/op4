
**This fork is Community Supported!**
------------------------

We **appreciate** all help! Anywhere we get it! 
To get involved join us on discord or Contribute to the project with code or help support the coders by donating.

- https://discord.gg/zWSnqJ6rKD

- Donating helps me dedicate more time and effort into this project. It also gives me more time to make for helping people.

- **Help** me get a **Comma 3** without selling my **Comma 2**. **Reason:** I will need to support both **Comma 2** and **Comma 3** users. I will try to support **Comma 2** users for as **long** as I can even if comma drops support!

<a href="https://www.paypal.com/donate?business=NRFAJ6FYRLT2Y&no_recurring=0&item_name=Contribute+to+help+progress+JPR%27s+HKG+Fork&currency_code=USD" 
target="_blank">
<img src="https://www.paypalobjects.com/en_US/GB/i/btn/btn_donateCC_LG.gif" alt="PayPal this" 
title="PayPal – The safer, easier way to pay online!" border="0" />
</a>

**Please join our Discord!**
---------------

https://discord.gg/zWSnqJ6rKD

**Submit issues on the issues page in a detailed report on GitHub or for quick help post in issues channel on our discord server.**

**Longitudinal Info**
----------------------

This fork has full long control for all HKG and Harnessless for older 2015 & 2016 & 2017 Genesis G80(Fixed SMDPS) without any radar harness mod. All other cars should require scc to be moved to bus 2 from bus 0 for full long control.

**Reach out to johnpr#5623 on discord too buy a radar harness. MDPS harnesses are available for sale with a 1 1/2 week lead time. Software to control it is currently WORKING and SUPPORTED!!**

**Liability**
------------

**It is open source and inherits MIT license.  By installing this software you accept all responsibility for anything that might occur while you use it.  All contributors to this fork are not liable.**  <b>Use at your own risk.</b>

**By using this software you are responsible for anything that occurs while OpenPilot is engaged or disengaged. Be ready to take over at any moment. Fork maintainers assumes no liability for your use of this software and any hardware.**

***Open Street Maps!***
---------------------
   - Speed limit wrong or missing? Contribute to Open Street Maps in your area! https://www.openstreetmap.org
   - Special Thanks to the Move-Fast Team for all the help and hard work with OSM!
   - With an active internet connection, and HKG Long, OpenPilot can plan ahead using vision and map data to slow for curves and adjust the longitudinal plan for speed limit and other factors.
   - Cruise speed does not adjust SCC max set speed, it instead adjusts longitudinal plan. So for it to work set max SCC speed higher than speed limit. To override speed limit tap on speed limit symbol on screen in top left corner next to max speed.
   - All Settings are under `Toggles` in `Settings`.
   - We have the correct DBC for newer Hyundais(2019+ and have built in navigation) to pull speed limit information from head unit over can bus, There is a toggle to do this `Pull Hyundai Navigation Speed Limit`. O.S.M. will use both database and car head unit input to decide speed limit.
   - https://www.youtube.com/watch?v=hTuvA6o6gjY

***SPAS***
----------
   - SPAS currently only **Supports EMS 366** EMS 11 is being worked on reach out to @johnpr#5623 on Discord to help!
   - Sends parking assist messages up to 41mph, With safety code to do the following and more, Rate limit, Override disengage, OpenPilot Correctly! handles all 8 states of MDPS_stat. OpenPilot can understand all MDPS faults and react accordingly.
   - OpenPilot disables on override.
   - Overried driver torque thresehold is 0.25 nm and is set in "carcontroller.py".
   - Openpilot takes into account and handels all 8 states in the correct order.
   - Openpilot correctly handles all MDPS faults.
   - Openpilot handels switch from SPAS to LKAS and back correctly, not to spam if hovering around 41mph.
   - SPAS has a Delta V rate limit on the steering thats speed corralated located in "carcontroller.py".
   - Max SPAS steering angle is set in "carcontroller.py" .
   - SPAS to LKAS switch speed is 41mph and SHOULD NOT BE SET HIGHER! This causes a wobble. SPAS to LKAS switch speed can be lowerd in "carcontroller.py".
   - https://www.youtube.com/watch?v=9U3gntnhbvM
   - https://www.youtube.com/watch?v=hTuvA6o6gjY

***HKG Long control toggle. (radar + vision)***
-----------------------------------------------
   - When toggled on, replaces the default Hyundai / Kia / Genesis factory longitudinal control system (SCC) with the openpilot system. May be useful for systems that don’t currently HAVE SCC but can support SCC via openpilot when programmed to another car variant(same model) that has radar..
   - Lead markers are not available unless you have HKG long.
   - Radar harness needed except for 2015 - 2016 Genesis.


***RetroPilot***
----------------

This fork uses RetroPilot for logging and online services.  https://api.retropilot.org/useradmin

Make sure "Upload Raw Logs" and "Enable Logger / Uploader" are both ON for this to work. 

**Hardware**
------------
The **Comma 3** should have significant performance improvements over Comma 2. **Comma 3 is untested** on this fork. I currently do not own a Comma 3 or have one on order. Help me reach my goal of rasing enough money for a Comma 3 to support users who upgraded. 

<a href="https://www.paypal.com/donate?business=NRFAJ6FYRLT2Y&no_recurring=0&item_name=Contribute+to+help+progress+JPR%27s+HKG+Fork&currency_code=USD" 
target="_blank">
<img src="https://www.paypalobjects.com/en_US/GB/i/btn/btn_donateCC_LG.gif" alt="PayPal this" 
title="PayPal – The safer, easier way to pay online!" border="0" />
</a>

The **Comma 2** has **POOR** performance with logging and uploader enabled so it's disabled by default. You can change that in `Settings` under `Community`.

- **MDPS Harnesses** are availaible for sale if you have the newer style plug. Contact johnpr#5623 on discord for more information.

- **Radar Harnesses** for Kia Stinger & G70 are for sale. Contact johnpr#5623 on discord for more information.

**Notes**
---------

Make sure to **shut off** auto start stop or you will get steering temporarily unavailable if the engine shuts off.

**Screen Recordings** 
- Saved to. `/storage/emulated/0/videos`

**Features**
------------

**Click** any of the settings to get a breif description in OpenPilot settings.

**Loading Logo**

The loading logo is automatically set to your HKG cars brand after the first boot, first car start, first reboot, and resets on update.

***nTune***
- nTune Auto Tunes lateral steering.

Run **nTune** after 30 - 50 miles of driving. It will autotune lateral control. Use this command `cd selfdrive && python ntune.py` or use the button in `Settings` under `Device`. (make sure your not driving!)

**Delete UI Screen Recordings button in `Settings` under `Device`.**

**Toggles**

- Toggles are in `Settings` under `Community`.

***Cluster Speed***

   - Uses the speed of the gauge cluster instead GPS speed.

***LDWS toggle***

   - under `Community` in `Settings`. For cars with LDWS but not SCC.

***Show Debug UI***

   - I feel like you should understand what “debugging” and a “UI” are before you can use openpilot

***Use SMDPS Harness***

   - Use of MDPS Harness to enable openpilot steering down to 0 MPH

***Built in TPMS Alerts***

   - An alert is displayed showing what tire and pressure is low.

***Stop Screen Capture on disengage toggle.***

***On screen blinkers and blind spot alerts.***

***Enable Lane Change Assist***

  - allows openpilot to change lanes. Driver is responsible for ensuring that it is SAFE to change lanes. Requires signal, and steering wheel nudge.

***Auto Lane Change with Blind spot monitoring toggle (No Nudge).***
  - Same as the original, now with 100% less nudge.

***Sync Speed on Gas Press***

  - openpilot will sync cruise control set speed to match last attained speed automatically

***Make sure to reboot with toggle changes.***

Then give it a spin.

***Behavior Notes***
--------------------

OpenPilot HKG Long will not see totally stopped cars yet until E2E comes in 0.9 so do not trust it to see and stop for a COMPLETLY stopped car.

If Collision Warning is beeping at you OpenPilot has calculated it can't stop quick enough due to safety limitations on unintentional braking. Please apply brakes to avoid collision.

***Install***
------------

***0.8.8 From Setup***
---------------------

**Test**

Put this url during setup for Test `https://smiskol.com/fork/Circuit-Pro/test`

**Stable**

Put this url during setup for Stable `https://smiskol.com/fork/Circuit-Pro/stable`

***0.8.8 From SSH***
---------------------

**Test**

`git remote add circuit-pro https://github.com/Circuit-Pro/openpilot && git fetch --all && git checkout test && reboot`

**Stable**

`git remote add circuit-pro https://github.com/Circuit-Pro/openpilot && git fetch --all && git checkout stable && reboot`

***If you installed from SSH***

- make sure too run `rm /data/params/d/DongleId` to reset your dongle ID.




**This is based on xx979xx & Neokii's & crwusiz's fork and is tuned best for Genesis G70, Kia Stinger, and works on others.. Please submit a tune to our discord if you find a better one.**

**Software Sources!**
----------------------------

https://github.com/neokii/op4
=======

https://github.com/xx979xx/openpilot

It is open source and inherits MIT license.  By installing this software you accept all responsibility for anything that might occur while you use it.  All contributors to this fork are not liable.  <b>Use at your own risk.</b>


------------------------------------------------------


![](https://user-images.githubusercontent.com/37757984/127420744-89ca219c-8f8e-46d3-bccf-c1cb53b81bb1.png)

Table of Contents
=======================

* [What is openpilot?](#what-is-openpilot)
* [Running in a car](#running-in-a-car)
* [Running on PC](#running-on-pc)
* [Community and Contributing](#community-and-contributing)
* [User Data and comma Account](#user-data-and-comma-account)
* [Safety and Testing](#safety-and-testing)
* [Directory Structure](#directory-structure)
* [Licensing](#licensing)

---

What is openpilot?
------

[openpilot](http://github.com/commaai/openpilot) is an open source driver assistance system. Currently, openpilot performs the functions of Adaptive Cruise Control (ACC), Automated Lane Centering (ALC), Forward Collision Warning (FCW) and Lane Departure Warning (LDW) for a growing variety of [supported car makes, models and model years](docs/CARS.md). In addition, while openpilot is engaged, a camera based Driver Monitoring (DM) feature alerts distracted and asleep drivers. See more about [the vehicle integration and limitations here](docs/INTEGRATION.md).

<table>
  <tr>
    <td><a href="https://www.youtube.com/watch?v=mgAbfr42oI8" title="YouTube" rel="noopener"><img src="https://i.imgur.com/kAtT6Ei.png"></a></td>
    <td><a href="https://www.youtube.com/watch?v=394rJKeh76k" title="YouTube" rel="noopener"><img src="https://i.imgur.com/lTt8cS2.png"></a></td>
    <td><a href="https://www.youtube.com/watch?v=1iNOc3cq8cs" title="YouTube" rel="noopener"><img src="https://i.imgur.com/ANnuSpe.png"></a></td>
    <td><a href="https://www.youtube.com/watch?v=Vr6NgrB-zHw" title="YouTube" rel="noopener"><img src="https://i.imgur.com/Qypanuq.png"></a></td>
  </tr>
  <tr>
    <td><a href="https://www.youtube.com/watch?v=Ug41KIKF0oo" title="YouTube" rel="noopener"><img src="https://i.imgur.com/3caZ7xM.png"></a></td>
    <td><a href="https://www.youtube.com/watch?v=NVR_CdG1FRg" title="YouTube" rel="noopener"><img src="https://i.imgur.com/bAZOwql.png"></a></td>
    <td><a href="https://www.youtube.com/watch?v=tkEvIdzdfUE" title="YouTube" rel="noopener"><img src="https://i.imgur.com/EFINEzG.png"></a></td>
    <td><a href="https://www.youtube.com/watch?v=_P-N1ewNne4" title="YouTube" rel="noopener"><img src="https://i.imgur.com/gAyAq22.png"></a></td>
  </tr>
</table>


Running in a car
------

To use openpilot in a car, you need four things
* This software. It's free and available right here.
* One of [the 140+ supported cars](docs/CARS.md). We support Honda, Toyota, Hyundai, Nissan, Kia, Chrysler, Lexus, Acura, Audi, VW, and more. If your car is not supported, but has adaptive cruise control and lane keeping assist, it's likely able to run openpilot.
* A supported device to run this software. This can be a [comma two](https://comma.ai/shop/products/two), [comma three](https://comma.ai/shop/products/three), or if you like to experiment, a [Ubuntu computer with webcams](https://github.com/commaai/openpilot/tree/master/tools/webcam).
* A way to connect to your car. With a comma two or three, you need only a [car harness](https://comma.ai/shop/products/car-harness). With an EON or PC, you also need a [black panda](https://comma.ai/shop/products/panda).

We have detailed instructions for [how to install the device in a car](https://comma.ai/setup).

Running on PC
------

To run and develop openpilot, you do not need a car or any special hardware.

The easiest way to get started is [with the CARLA simulator](tools/sim/README.md). This allows openpilot to drive around a virtual car on your Ubuntu machine. The whole setup should only take a few minutes.

You can also plot logs from a device and replay a drive. See [the tools README](tools/README.md) for more information.

Community and Contributing
------

openpilot is developed by [comma](https://comma.ai/) and by users like you. We welcome both pull requests and issues on [GitHub](http://github.com/commaai/openpilot). Bug fixes and new car ports are encouraged. Check out [the contributing docs](docs/CONTRIBUTING.md).

You can add support for your car by following guides we have written for [Brand](https://blog.comma.ai/how-to-write-a-car-port-for-openpilot/) and [Model](https://blog.comma.ai/openpilot-port-guide-for-toyota-models/) ports. Generally, a car with adaptive cruise control and lane keep assist is a good candidate. [Join our Discord](https://discord.comma.ai) to discuss car ports: most car makes have a dedicated channel.

Want to get paid to work on openpilot? [comma is hiring](https://comma.ai/jobs/).

And [follow us on Twitter](https://twitter.com/comma_ai).

User Data and comma Account
------

By default, openpilot uploads the driving data to our servers. You can also access your data through [comma connect](https://connect.comma.ai/). We use your data to train better models and improve openpilot for everyone.

openpilot is open source software: the user is free to disable data collection if they wish to do so.

openpilot logs the road facing camera, CAN, GPS, IMU, magnetometer, thermal sensors, crashes, and operating system logs.
The driver facing camera is only logged if you explicitly opt-in in settings. The microphone is not recorded.

By using openpilot, you agree to [our Privacy Policy](https://connect.comma.ai/privacy). You understand that use of this software or its related services will generate certain types of user data, which may be logged and stored at the sole discretion of comma. By accepting this agreement, you grant an irrevocable, perpetual, worldwide right to comma for the use of this data.

Safety and Testing
----

* openpilot observes ISO26262 guidelines, see [SAFETY.md](docs/SAFETY.md) for more details.
* openpilot has software in the loop [tests](.github/workflows/selfdrive_tests.yaml) that run on every commit.
* The code enforcing the safety model lives in panda and is written in C, see [code rigor](https://github.com/commaai/panda#code-rigor) for more details.
* panda has software in the loop [safety tests](https://github.com/commaai/panda/tree/master/tests/safety).
* Internally, we have a hardware in the loop Jenkins test suite that builds and unit tests the various processes.
* panda has additional hardware in the loop [tests](https://github.com/commaai/panda/blob/master/Jenkinsfile).
* We run the latest openpilot in a testing closet containing 10 comma devices continuously replaying routes.

Directory Structure
------
    .
    ├── cereal              # The messaging spec and libs used for all logs
    ├── common              # Library like functionality we've developed here
    ├── docs                # Documentation
    ├── opendbc             # Files showing how to interpret data from cars
    ├── panda               # Code used to communicate on CAN
    ├── phonelibs           # External libraries
    ├── pyextra             # Extra python packages not shipped in NEOS
    └── selfdrive           # Code needed to drive the car
        ├── assets          # Fonts, images, and sounds for UI
        ├── athena          # Allows communication with the app
        ├── boardd          # Daemon to talk to the board
        ├── camerad         # Driver to capture images from the camera sensors
        ├── car             # Car specific code to read states and control actuators
        ├── common          # Shared C/C++ code for the daemons
        ├── controls        # Planning and controls
        ├── debug           # Tools to help you debug and do car ports
        ├── locationd       # Precise localization and vehicle parameter estimation
        ├── logcatd         # Android logcat as a service
        ├── loggerd         # Logger and uploader of car data
        ├── modeld          # Driving and monitoring model runners
        ├── proclogd        # Logs information from proc
        ├── sensord         # IMU interface code
        ├── test            # Unit tests, system tests, and a car simulator
        └── ui              # The UI

Licensing
------

openpilot is released under the MIT license. Some parts of the software are released under other licenses as specified.

Any user of this software shall indemnify and hold harmless Comma.ai, Inc. and its directors, officers, employees, agents, stockholders, affiliates, subcontractors and customers from and against all allegations, claims, actions, suits, demands, damages, liabilities, obligations, losses, settlements, judgments, costs and expenses (including without limitation attorneys’ fees and costs) which arise out of, relate to or result from any use of this software by user.

**THIS IS ALPHA QUALITY SOFTWARE FOR RESEARCH PURPOSES ONLY. THIS IS NOT A PRODUCT.
YOU ARE RESPONSIBLE FOR COMPLYING WITH LOCAL LAWS AND REGULATIONS.
NO WARRANTY EXPRESSED OR IMPLIED.**

---

<img src="https://d1qb2nb5cznatu.cloudfront.net/startups/i/1061157-bc7e9bf3b246ece7322e6ffe653f6af8-medium_jpg.jpg?buster=1458363130" width="75"></img> <img src="https://cdn-images-1.medium.com/max/1600/1*C87EjxGeMPrkTuVRVWVg4w.png" width="225"></img>

[![openpilot tests](https://github.com/commaai/openpilot/workflows/openpilot%20tests/badge.svg?event=push)](https://github.com/commaai/openpilot/actions)
[![Total alerts](https://img.shields.io/lgtm/alerts/g/commaai/openpilot.svg?logo=lgtm&logoWidth=18)](https://lgtm.com/projects/g/commaai/openpilot/alerts/)
[![Language grade: Python](https://img.shields.io/lgtm/grade/python/g/commaai/openpilot.svg?logo=lgtm&logoWidth=18)](https://lgtm.com/projects/g/commaai/openpilot/context:python)
[![Language grade: C/C++](https://img.shields.io/lgtm/grade/cpp/g/commaai/openpilot.svg?logo=lgtm&logoWidth=18)](https://lgtm.com/projects/g/commaai/openpilot/context:cpp)
[![codecov](https://codecov.io/gh/commaai/openpilot/branch/master/graph/badge.svg)](https://codecov.io/gh/commaai/openpilot)
