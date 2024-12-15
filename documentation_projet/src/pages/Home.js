import React from "react";
import LeftSidebar from "../components/LeftSidebar.js";

// Icons
import { FontAwesomeIcon } from "@fortawesome/react-fontawesome";
import { faHome, faCogs, faMicrochip, faDesktop, faChartBar, faBook} from "@fortawesome/free-solid-svg-icons";

export function Home() {
  const activeChild = "Accueil";

  return (
    <div className="flex h-screen">
      {/* Left Sidebar */}
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

      {/* Main content area */}
      <div className="flex flex-col flex-1 overflow-y-auto">

        {/* Main page content */}
        <div className="flex flex-col flex-1 p-8 space-y-8">
          {/* Project Details Section */}
          <div className="bg-white shadow-lg rounded-lg p-6 border-l-4 border-blue-500">
            <h1 className="text-xl font-extrabold text-blue-600 mb-4">Objectif du Projet</h1>
            <p className="text-gray-700">
              L'objectif de ce projet est de contrôler la position d'un servomoteur à l'aide des déformations mesurées par des jauges de contrainte. Deux modes de fonctionnement ont été mis en place :
            </p>
            <ul className="list-disc pl-6 mt-4 text-gray-700">
              <li>
                <strong>Mode manuel :</strong> l'utilisateur définit directement l'angle du servomoteur pour observer son positionnement.
              </li>
              <li>
                <strong>Mode automatique :</strong> la position du servomoteur est ajustée automatiquement en fonction de l'effort appliqué et mesuré par la déformation des jauges.
              </li>
            </ul>
            <p className="mt-2 text-gray-700">
              Dans les deux modes, un affichage visuel sous forme de <strong>schéma radar</strong> ou de <strong>flèche directionnelle</strong> permet de visualiser l'angle atteint par le servomoteur.
            </p>
            <p className="mt-2 text-gray-700">
              Concernant les jauges de déformation, nous avons réalisé :
            </p>
            <ul className="list-disc pl-6 mt-2 text-gray-700">
              <li>Un <strong>graphe</strong> représentant la tension de sortie des jauges.</li>
              <li>Un affichage des données de la <strong>masse appliquée</strong>,la <strong>force exercée</strong> et la <strong>déformation</strong> observée.</li>
              <li>La mise en place d'un <strong>schéma 3D</strong> illustrant la déformation de la lame sous l'effet des forces appliquées.</li>
            </ul>
            <p className="mt-2 text-gray-700">
              Ce projet offre ainsi une solution complète pour analyser, visualiser et contrôler des déformations mécaniques dans des applications de flexion.
            </p>
          </div>


          {/* Website Purpose Section */}
          <div className="bg-white shadow-lg rounded-lg p-6 border-l-4 border-purple-500">
            <h2 className="text-xl font-extrabold text-purple-600 mb-4">Objectif du Site Web</h2>
            <p className="text-gray-700">
              Ce site web sert de documentation complète pour le projet. Il fournit des détails techniques, des explications sur les méthodes utilisées, et des résultats obtenus. Il est conçu pour être une référence pour les étudiants et les chercheurs travaillant sur des projets similaires.
            </p>
          </div>

          {/* Team Section */}
          <div className="bg-white shadow-lg rounded-lg p-6 border-l-4 border-green-500">
            <h2 className="text-xl font-extrabold text-green-600 mb-4">Réalisé par</h2>
            <ul className="list-disc pl-6 text-gray-700">
              <li>CHOUBRI Douae</li>
              <li>BOUGHANMI Roua</li>
              <li>MAMBOU TSAGA Duval</li>
              <li>LEMAIRE Rodrigue</li>
            </ul>
          </div>

        </div>
      </div>
    </div>
  );
}

export default Home;
