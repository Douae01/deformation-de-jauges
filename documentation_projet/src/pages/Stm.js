import React from 'react';
import LeftSidebar from '../components/LeftSidebar';

// Fonts and Icons
import { FontAwesomeIcon } from "@fortawesome/react-fontawesome";
import { faHome, faCogs, faMicrochip, faDesktop, faChartBar, faBook } from "@fortawesome/free-solid-svg-icons";

export const Stm = () => {
  const activeChild = 'STM32';

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

      {/* Content Section */}
      <div className="flex-1 bg-white shadow-md rounded-lg p-6 overflow-auto">
        <h1 className="text-3xl font-bold mb-6 text-gray-800">Programmation STM32</h1>

      {/* Section 1: Introduction */}
      <section className="mb-6">
        <h2 className="text-2xl font-semibold mb-4 text-gray-700">Introduction</h2>
        <p>
          Dans ce projet, la carte STM32 est utilisée pour contrôler un servomoteur en fonction de la déformation d'une lame. Le système possède deux modes :
        </p>
        <ul className="list-disc list-inside mt-2 ml-4">
          <li><strong>Mode Manuel :</strong> L'utilisateur définit directement l'angle de rotation du servomoteur.</li>
          <li>
            <strong>Mode Automatique :</strong> La position du servomoteur est ajustée automatiquement en fonction des données mesurées par des jauges de déformation.
          </li>
        </ul>
        <p className="mt-4">
          En complément, le système permet d'afficher en temps réel la <strong>tension</strong> mesurée par les jauges de déformation. Cette tension est utilisée pour :
        </p>
        <ul className="list-disc list-inside mt-2 ml-4">
          <li>
            Calculer la <strong>masse</strong> appliquée sur la lame à partir d'une relation linéaire entre la tension et la masse.
          </li>
          <li>
            Déterminer la <strong>force</strong> exercée sur la lame selon la formule : 
            <span className="font-mono bg-gray-100 p-1 rounded">F = m * g</span>, où <strong>g</strong> est l'accélération de la pesanteur standard.
          </li>
          <li>
            Évaluer la <strong>déformation</strong> de la lame en fonction de la force appliquée et afficher les résultats obtenus.
          </li>
        </ul>
        <p className="mt-4">
          Ces données sont non seulement utilisées pour contrôler le servomoteur dans le mode automatique, mais aussi pour une analyse visuelle détaillée grâce à des graphiques et des représentations interactives.
        </p>
      </section>


        {/* Section 4: Tâches Préliminaires */}
        <section>
          <h2 className="mt-2 text-2xl font-semibold mb-4 text-gray-700">Tâches Préliminaires Validées</h2>
          <p>
            Avant d'entamer le projet, les trois tâches demandées dans le cadre du module <strong>IPS</strong> à l'ENIB ont été réalisées et validées :
          </p>
          <ul className="list-disc list-inside mt-2 ml-4">
            <li>
              <strong>Tâche 1 :</strong> Création d'une tâche périodique (1 seconde et 10 ms) validée grâce à un oscilloscope.
            </li>
            <li>
              <strong>Tâche 2 :</strong> Acquisition d'une donnée analogique, conversion en tension (via un potentiomètre), et vérification avec un oscilloscope.
            </li>
            <li>
              <strong>Tâche 3 :</strong> Transmission des données via UART sur USB pour visualisation sur PC.
            </li>
          </ul>
          <p className="mt-2">
            Ces tâches ont servi de base pour le développement du projet, notamment pour la gestion des jauges et du servomoteur.
          </p>
        </section>

        {/* Section 2: Traitement des Données */}
        <section className="mt-6 mb-6">
          <h2 className="text-2xl font-semibold mb-4 text-gray-700">Traitement des Données</h2>
          <p>
            La tension <strong>Vm</strong> mesurée par le circuit électrique est acquise via l'ADC (Convertisseur Analogique-Numérique) de la STM32. La conversion est réalisée à l'aide de la formule suivante :
          </p>
          <div className="bg-gray-100 p-4 rounded-md font-mono text-sm mt-2">
            voltage = Vm * (3.3 / 4095.0); <br />
          </div>
          <p>où <strong>3.3</strong> représente la tension de référence et <strong>4095</strong> correspond à la résolution du convertisseur ADC sur 12 bits.</p>
          <p className="mt-4">
            Cette tension est ensuite utilisée pour estimer la <strong>masse</strong> dans un intervalle de 0 à 100g, à l'aide de l'équation linéaire :
          </p>
          <div className="bg-gray-100 p-4 rounded-md font-mono text-sm mt-2">
            y = ax + b
          </div>
          <p className="mt-4">
            Enfin, cette masse est convertie en une <strong>valeur d'angle</strong> entre 0° et 180°, en calculant la force appliquée sur la lame :
          </p>
          <div className="bg-gray-100 p-4 rounded-md font-mono text-sm mt-2">
            F = m * g
          </div>
          <p className="mt-4">
            Le servomoteur est commandé par un signal PWM généré par la STM32, permettant un ajustement précis de sa position.
          </p>
        </section>
      </div>
    </div>
  );
};
