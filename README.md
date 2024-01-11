# EXO workspace

Développement du software sur EPS32 pour l'exosquelette de puissance HERMESETS en vue de la compétition ACE. 

## Installation

L'environnement de développement est Visual Studio Code sous Windows 10.

- Setup de l'IDE

Quick tutorial pour l'installation de ESP-IDF pour ESP32 sous VS code : [Getting started ESP-IDF VScode](https://www.youtube.com/watch?v=Lc6ausiKvQM&feature=youtu.be&ab_channel=EspressifSystems)

- Clone du repo git

`git clone https://github.com/ClubSynapsETSOrganisationnalGithub/Exo.git/Exo`

Go in the repo main, select the target (EPS32-S2), select your port COM, build, flash and monitor the results.

## Features 

- IMU : Récupération des 3 axes de l'accéléromètre et des 3 axes du gyroscope. Library homemade issu de [Grove_IMU_10DOF_v2.0](https://github.com/Seeed-Studio/Grove_IMU_10DOF_v2.0)
- Thread implémentation : l'accès aux senseurs IMU se fait par un thread en parallèle qui roule indépendamment de la fonction main. 
- ESC : Electronic speed controller from flipsky [Mini FSESC4.20 50A base on VESC](https://flipsky.net/collections/electronic-products/products/mini-fsesc4-20-50a-base-on-vesc-widely-used-in-eskateboard-escooter-ebike) and watch demo for more information [DIY ebike using flipsky tech](https://www.youtube.com/watch?v=3ov6Q745u9g).

## How to build the project in your VS Code

1. All custom code that is used in the project must be put in the inc/ and src/ folder in the top folder C:\SynapseDev\EXO, then inside your project you fetch the code using :
**../inc/file_name.h**.

2. Using ESP-IDF call the command : **ESP-IDF: New Project** and use the following criteria : 
    - Name the projet **EXO_application**
    - Choose the repository : **C:\SynapseDev\EXO** as the projet reposity
    - For the component library use the path : **C:\Users\username\esp\esp-idf\components**

3. Une fois le projet créé, VS Code vous demandera si vous vouler ouvrir une nouvelle fenêtre pour le projet crée, vous devez dire oui.

4. Une fois que vous avez ouvert une nouvelle fenêtre, vous pouvez compiler le projet en suivant les instructions de Expressif [VS Code extension](https://github.com/espressif/vscode-esp-idf-extension/blob/master/docs/tutorial/basic_use.md). 

## Servo motor information 

The servo used in the project are the LD-260MD Digital Servo. You can find the zip file with the information of the servo at the following link [Hiwonder Google Drive](https://drive.google.com/drive/folders/1Bgf1HGrfhB8N8XIxlRpz-U9_2oxVurDv) et sur le sharepoint de SynapsÉTS avec le lien suivant [Lien sharepoint vers le dossier](https://etsmtl365.sharepoint.com/:u:/r/sites/msteams_77c024/Documents%20partages/Project%20Exo/2022-2023/Documentation/Datasheets/LD-260MG%20Digital%20Servo.7z?csf=1&web=1&e=BtieLK).

# The prototype for ACE competition in Michigan State University

![IMG_0915](https://github.com/VictorRios-hub/Exoskeleton_SynapsETS/assets/99796369/8622771e-c5c6-4af5-8cd3-5cf3873482b9)
![Afficher les photos récentes](https://github.com/VictorRios-hub/Exoskeleton_SynapsETS/assets/99796369/aef0ac55-05d0-448d-9ced-0921d116da57)

