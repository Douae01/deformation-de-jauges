# Déformation de Jauges

## Description du Projet

L'objectif de ce projet est de contrôler la position d'un servomoteur à l'aide des déformations mesurées par des jauges de contrainte. Deux modes de fonctionnement ont été mis en place :

1. **Mode manuel** : l'utilisateur définit directement l'angle du servomoteur pour observer son positionnement.
2. **Mode automatique** : la position du servomoteur est ajustée automatiquement en fonction de l'effort appliqué et mesuré par la déformation des jauges.

Dans les deux modes, un affichage visuel sous forme de schéma radar ou de flèche directionnelle permet de visualiser l'angle atteint par le servomoteur.

### Fonctionnalités des jauges de déformation

- Un graphe représentant la tension de sortie des jauges.
- Un affichage des données suivantes :
  - Masse appliquée
  - Force exercée
  - Déformation observée
- Un schéma 3D illustrant la déformation de la lame sous l'effet des forces appliquées.

Ce projet offre ainsi une solution complète pour analyser, visualiser et contrôler des déformations mécaniques dans des applications de flexion.

---

## Prérequis

Pour exécuter ce projet, vous aurez besoin de :

1. **STM32CubeIDE** : pour exécuter le code embarqué sur la carte STM32.
2. **Node.js** : pour lancer l'interface utilisateur développée avec Electron.

---

## Instructions d'Exécution

### Étape 1 : Exécuter le code STM32

1. Ouvrez le projet embarqué dans **STM32CubeIDE**.
2. Compilez et téléchargez le code sur la carte STM32 connectée.

### Étape 2 : Lancer l'interface utilisateur (IHM)

1. Accédez au répertoire contenant l'interface utilisateur.
2. Installez les dépendances nécessaires avec la commande suivante :

   ```bash
   npm install
3. Lancez l'interface utilisateur en exécutant :

   ```bash
   npm run electron-dev
   
---

## Documentation

Pour plus d'informations sur la conception, les fonctionnalités et l'implémentation du projet, veuillez consulter la [documentation complète](https://doc-proj.onrender.com/).


