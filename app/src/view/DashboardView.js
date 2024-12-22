import React from 'react';
import logo from "../assets/enib.png";

const DashboardView = () => {
  return (
    <div className="flex flex-col items-center justify-center w-full min-h-screen">
      <div className="relative w-full bg-white rounded-lg shadow-lg p-10">
        <div className="absolute top-4 left-4">
          <img src={logo} alt="ENIB Logo" className="h-16 w-18" />
        </div>
        <div className="flex flex-col items-center">
          <h1 className="text-4xl font-bold mb-6">Accueil</h1>
          <h2 className="text-2xl mb-8 text-center">
            Cette interface utilisateur a été conçue par des étudiants de 4ème année de l'ENIB :
            <br/>
            CHOUBRI Douae, MAMBOU Duval, BOUGHANMI Roua, et LEMAIRE Rodrigue.
          </h2>
          <div className="text-left w-full">
            <h1 className="text-3xl font-semibold mb-4">Description de l'interface :</h1>
            <p className="text-lg leading-relaxed mb-6">
              Cette page sert d'accueil d'interface. Concernant la page de Contrôle Moteur et Jauges vous permet de commander le servomoteur en mode manuel ou automatique,
              tout en visualisant en temps réel un graphe de tension des jauges accompagné d'un ensemble de données
              relatives à la masse, à la déformation, et à la force appliquée sur la lame à jauges. Un schéma 3D est inclus,
              offrant une vue presque identique à la lame en temps réel.
            </p>
          </div>
          <h3 className="text-center text-2xl font-semibold text-green-600">
            Bonne découverte de notre IHM !
          </h3>
        </div>
      </div>
    </div>
  );
};

export default DashboardView;
