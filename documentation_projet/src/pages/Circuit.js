import React from 'react';
import LeftSidebar from '../components/LeftSidebar';
import PCB from "../assets/images/pcb.png";
import circuit from "../assets/images/schema_circuit.png";

// Fonts and Icons
import { FontAwesomeIcon } from "@fortawesome/react-fontawesome";
import { faHome, faCogs, faMicrochip, faDesktop, faChartBar, faBook } from "@fortawesome/free-solid-svg-icons";

export const Circuit = () => {
  const activeChild = 'Circuit';

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

      {/* Main Content */}
      <div className="flex-grow p-6 overflow-y-auto">
        <h1 className="text-3xl font-bold mb-6 text-gray-800">Circuit électrique</h1>
        {/* Images Section */}
        <div className="flex flex-col items-center gap-6 mb-6 mt-2">
          {/* Schema Circuit */}
          <div className="w-full">
            <h2 className="text-2xl font-semibold mb-4 text-gray-700">Schéma du Circuit</h2>
            <img 
              src={circuit}
              alt="Schéma du Circuit" 
              className="border rounded-lg shadow-lg h-10px m-auto"
            />
          </div>

          {/* PCB */}
          <div className="w-full justify-center items-center">
            <h2 className="text-2xl font-semibold mb-4 text-gray-700">Schéma du PCB</h2>
            <img 
              src={PCB} 
              alt="Schéma du PCB" 
              className="border rounded-lg shadow-lg h-10px m-auto"
            />
          </div>
        </div>

        <h2 className="text-2xl font-semibold mb-4 text-gray-700">Description du Circuit</h2>
        {/* Text Description */}
        <div className="bg-gray-100 p-4 rounded-lg shadow-lg">
          <p className="mb-2">
            Ce circuit utilise quatre jauges de déformation connectées en pont de Wheatstone 
            pour mesurer de faibles variations de résistance. La sortie du pont de Wheatstone 
            permet d'obtenir une tension proportionnelle à la déformation, exprimée par l'équation du pont 
            suivante :
          </p>
          <p className="mb-2 text-center font-semibold">
            <em>Vm = (ΔR / R) * Ve</em>
          </p>
          <p className="mb-2">
            Cependant, la tension obtenue directement à la sortie du pont de Wheatstone est très faible et sensible aux perturbations, 
            notamment en raison des variations bruitées des jauges observées lors des tests initiaux. 
          </p>
          <p className="mb-2">
            Pour résoudre ce problème et garantir une précision optimale, nous utilisons un amplificateur différentiel <strong>AD623</strong>, qui ajuste, amplifie et stabilise le signal avant son envoi vers la carte STM32 pour traitement.
          </p>
        </div>
      </div>
    </div>
  );
};
