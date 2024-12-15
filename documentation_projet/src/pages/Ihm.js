import React from 'react';
import LeftSidebar from '../components/LeftSidebar';

// Fonts and Icons
import { FontAwesomeIcon } from "@fortawesome/react-fontawesome";
import { faHome, faCogs, faMicrochip, faDesktop, faChartBar, faBook } from "@fortawesome/free-solid-svg-icons";

export const Ihm = () => {
  const activeChild = 'IHM';

  return (
    <div className="flex h-screen">
      {/* Sidebar */}
      <div className="w-1/6 flex-shrink-0">
        <LeftSidebar activeChild={activeChild}>
          <div href="/" name="Accueil" icon={<FontAwesomeIcon icon={faHome} />} />
          <div href="/Circuit" name="Circuit" icon={<FontAwesomeIcon icon={faCogs} />} />
          <div href="/Stm" name="STM32" icon={<FontAwesomeIcon icon={faMicrochip} />} />
          <div href="/Ihm" name="IHM" icon={<FontAwesomeIcon icon={faDesktop} />} />
          <div href="/Etude" name="Etudes" icon={<FontAwesomeIcon icon={faChartBar} />} />
          <div href="/Biblio" name="Bibliographie" icon={<FontAwesomeIcon icon={faBook} />} />
        </LeftSidebar>
      </div>
      <div className="flex-1 bg-white shadow-md rounded-lg p-6 overflow-auto">
        <h1 className="text-3xl font-bold mb-6 text-gray-800">Interface Homme-Machine (IHM)</h1>

      {/* Section 1: Introduction */}
      <section className="mb-6">
        <h2 className="text-2xl font-semibold mb-4 text-gray-700">Introduction</h2>
          <p>
            L'interface graphique développée permet d'interagir avec le système en toute simplicité. Elle est composée de deux sections principales dans tableau de bord:
          </p>
          <ul className="list-disc list-inside mt-2 ml-4">
            <li>
              <strong>Accueil :</strong> Présentation d'un résumé du projet avec les objectifs principaux ainsi que les noms des membres du groupe.
            </li>
            <li>
              <strong>Contrôle Moteur et Jauges :</strong> Section dédiée au contrôle du servomoteur et à l'affichage des données relevées par les jauges de déformation.
            </li>
          </ul>
          <p className="mt-4">
            Les différentes fonctionnalités de l'interface permettent de visualiser et de manipuler les résultats en temps réel pour une meilleure analyse et compréhension.
          </p>
        </section>

        {/* Section 2: Fonctionnalités Principales */}
        <section className="mb-8">
          <h2 className="text-2xl font-semibold text-gray-700 mb-4">Fonctionnalités Principales</h2>
          <p>
            L'interface offre les fonctionnalités suivantes :
          </p>
          <ul className="list-disc list-inside mt-2 ml-4">
            <li>Affichage d'un <strong>résumé</strong> du projet avec objectifs et équipe.</li>
            <li>Contrôle manuel et automatique du <strong>servomoteur</strong>.</li>
            <li>Représentation d'un schéma radar ou d'une flèche indiquant l'angle défini dans les deux modes.</li>
            <li>Visualisation en temps réel de <strong>tension</strong> reçue des jauges sous forme d'un graphe.</li>
            <li>Visualisation des données associées à la <strong>masse</strong>, à la <strong>force</strong> appliquée et à la <strong>déformation</strong> de la lame.</li>
            <li>Tentative d'affichage 3D de la déformation pour une meilleure représentation visuelle.</li>
          </ul>
        </section>

        {/* Section 4: Aperçu de l'Interface */}
        <section>
          <h2 className="text-2xl font-semibold text-gray-700 mb-4">Aperçu de l'Interface</h2>
          <p>Voici quelques aperçus de l'interface graphique développée :</p>
          <div className="grid grid-cols-2 gap-4 mt-4">
            <div className="bg-gray-200 p-4 rounded-md text-center">
              <p className="font-medium">Image 1 : Tableau de Bord et Accueil</p>
            </div>
            <div className="bg-gray-200 p-4 rounded-md text-center">
              <p className="font-medium">Image 2 : Contrôle Moteur et Jauges</p>
            </div>
          </div>
        </section>
      </div>
    </div>
  );
};
