import React from 'react';
import LeftSidebar from '../components/LeftSidebar';
import BF350 from '../assets/pdf/BF350-3AA.pdf'
import Jauges from '../assets/pdf/AN078-Strain_Gauge_Measurement-A_Tutorial.pdf'
import MG90S from '../assets/pdf/MG90S.pdf'
import Projet from '../assets/pdf/Sujet_projet.pdf'

// Fonts and Icons
import { FontAwesomeIcon } from "@fortawesome/react-fontawesome";
import { faHome, faCogs, faMicrochip, faDesktop, faChartBar, faBook } from "@fortawesome/free-solid-svg-icons";

export const Biblio = () => {
  const activeChild = 'Bibliographie';

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
        <h1 className="text-3xl font-bold mb-6 text-gray-800">Bibliographie</h1>
        
        <div className="flex flex-col gap-4">
          {/* Datasheet for Jauges */}
          <div className="border p-4 rounded-lg shadow-lg">
            <h2 className="text-2xl font-semibold mb-4 text-gray-700">Datasheet pour Jauges AN078 et BF350-3AA</h2>
            <p className="mb-2">
              Cette documentation couvre les spécifications des jauges de déformation utilisées dans le projet et documentation général.
            </p>
            <a 
              href={Jauges}
              target="_blank" 
              rel="noopener noreferrer" 
              className="text-blue-500 underline"
            >
              Télécharger le PDF des jauges
            </a>
            <br/>
            <a 
              href={BF350}
              target="_blank" 
              rel="noopener noreferrer" 
              className="text-blue-500 underline"
            >
              Télécharger le PDF BF350-3AA
            </a>
          </div>

          {/* Datasheet for Servo Moteur */}
          <div className="border p-4 rounded-lg shadow-lg">
            <h2 className="text-2xl font-semibold mb-4 text-gray-700">Datasheet pour Servo Moteur MG90S</h2>
            <p className="mb-2">
              Spécifications techniques et détails sur le fonctionnement du servomoteur utilisé.
            </p>
            <a 
              href={MG90S} 
              target="_blank" 
              rel="noopener noreferrer" 
              className="text-blue-500 underline"
            >
              Télécharger le PDF
            </a>
          </div>

          {/* Datasheet for Sujet Projet */}
          <div className="border p-4 rounded-lg shadow-lg">
            <h2 className="text-2xl font-semibold mb-4 text-gray-700">Sujet Projet IPS D-4</h2>
            <p className="mb-2">
              Informations générales et directives sur le sujet du projet.
            </p>
            <a 
              href={Projet}
              target="_blank" 
              rel="noopener noreferrer" 
              className="text-blue-500 underline"
            >
              Télécharger le PDF
            </a>
          </div>
        </div>
      </div>
    </div>
  );
};
