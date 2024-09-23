<h1 align="center">
UGV-CBRN: An Unmanned Ground Vehicle for Chemical, Biological, Radiological, and Nuclear Disaster Response
</h1>

<h3 align="center">
Simon Schwaiger<sup>*1,2</sup>, Lucas Muster<sup>*1,3</sup>, Georg Novotny<sup>1</sup>, Michael Schebek<sup>1</sup>, Wilfried Wöber<sup>1</sup>, Stefan Thalhammer<sup>1</sup> and Christoph Böhm<sup>1</sup>
</h3>

<i align="center">

<sup>\*</sup> Equal Contribution

This work was supported by the Austrian Research Promotion Agency (FFG) through the research project UGV-ABC-Probe (FFG project Call 2020) and the Austrian Armed Forces.

<sup>1</sup> University of Applied Sciences Technikum Wien, Faculty of Industrial Engineering, 1200 Vienna, Austria

<sup>2</sup> Graz University of Technology, Faculty of Computer Science and Biomedical Engineering, Institute of Software Technology, Inffeldgasse 16b/II, 8010 Graz, Austria

<sup>3</sup> University of Natural Resources and Life Sciences, Department of Biotechnology, Institute for Computational Biology, Muthgasse 18, 1190 Vienna, Austria

<a href="mailto:schwaige@technikumwien.at">schwaige@technikum-wien.at</a>,
<a href="mailto:muster@technikumwien.at">muster@technikum-wien.at</a>

</i>

<table align="center" style="border-collapse: collapse; width: 450px;">
  <tr>
    <td align="middle" style="border: none;">
      <a href="https://arxiv.org/pdf/2406.14385" style="color: white; font-size: 14pt;">
        <div style="background-color: #363636; border-radius: 50px; padding: 10px 20px;">
            <img src="img/document_icon.svg" height="14"> Paper
        </div>
      </a>
    </td>
    <td align="middle" style="border: none;">
      <a href="https://github.com/TW-Robotics/search-and-rescue-robot-2024/tree/main" style="color: white; font-size: 14pt;">
        <div style="background-color: #363636; border-radius: 50px; padding: 10px 20px;">
            <img src="img/logo_github.png" height="14"> Code
        </div>
      </a>
    </td>
    <td align="middle" style="border: none;">
      <a href="https://arxiv.org/abs/2406.14385" style="color: white; font-size: 14pt;">
        <div style="background-color: #363636; border-radius: 50px; padding: 10px 20px;">
            <img src="img/logo_arxiv.png" height="14"> arXiv
        </div>
      </a>
    </td>
  </tr>
</table>

<div style="max-width: 800px; margin: auto;", align="center">
  <img src="./img/NavigationDemo.gif" alt="Navigation Demo GIF" width="45%">
  <img src="./img/SamplingDemo.gif" alt="Sampling Demo GIF" width="45%">
</div>

<h2 align="center"> Abstract</h2>

<i align="center">

Robotic search and rescue (SAR) supports response teams by accelerating disaster assessment and by keeping operators away from hazardous environments. In the event of a chemical, biological, radiological, and nuclear (CBRN) disaster, robots are deployed to identify and locate radiation sources. Human responders then assess the situation and neutralize the danger. The presented system takes a step toward enhanced integration of robots into SAR teams. Integrating autonomous radiation mapping with semi-autonomous substance sampling and online analysis of the CBRN threat lets the human operator localize and assess the threat from a safe distance. Two LiDARs, an IMU, and a Geiger counter are used for mapping the surrounding area and localizing potential radiation sources. A mobile manipulator with six Degrees of Freedom manipulates valves and samples substances that are analyzed by an onboard Raman spectrometer. The human operator monitors the mission’s progression from a remote location defining target locations and directing the semi-autonomous manipulation processes. Diverse recovery behaviours aid robot deployment, system state monitoring, as well as recovery of hard- and software. Field tests showcase the capabilities of the presented system during trials at the CBRN disaster response
challenge European Robotics Hackathon (EnRicH).

</i>

***************************************

## Directory Structure

* `/operator`:
    Contains Docker-based workspace run on human operator's PC

* `/robot`:
    Contains workspace to run on robot's on-board PCs
    
    - `3dparty`:
        Third party methods integrated into the robot

        * `exploration`:
            Compose setup running explorer and switching between goalsources

        * `mapping`:
            Compose setup starting 2D and 3D mapping (rad mapping is on operator PC)

        * `perception`:
            Compose setup for projecting and filtering LiDAR measurements and reading preprocessed camera images from Jetson single board PC

        * `sensing`:
            Compose setup for reading LiDAR and IMU data

        * `systemd`:
            Blueprint for systemd setup that monitors each 3d party Compose workspace

    - `catkin_ws`:
        Local, non containerized catkin workspace for components that need to be run bare metal

    - `manipulation`:
        Compose setup for arm control

    - `navigation`:
        Compose setup for semi-autonomous behavior and fully autonomous navigation

    - `systemd`:
        Blueprint for systemd setup that monitors each Compose workspace

## Dataset Download

We provide datasets from practical field trials in Rosbag format. (Coming Soon)

## Citation

If you use this work in your research, please cite our paper:

```bibtex
@misc{SchwaigerMuster2024UGVCBRN,
    title               = {UGV-CBRN: An Unmanned Ground Vehicle for Chemical, Biological, Radiological, and Nuclear Disaster Response. \textit{arXiv preprint arXiv:2406.14385}}, 
    author              = {Simon Schwaiger and Lucas Muster and Georg Novotny and Michael Schebek and Wilfried Wöber and Stefan Thalhammer and Christoph Böhm},
    year                = {2024},
    url                 = {https://arxiv.org/abs/2406.14385}
}
```


