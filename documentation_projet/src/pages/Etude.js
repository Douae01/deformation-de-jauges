import React, { useState } from 'react';
import LeftSidebar from '../components/LeftSidebar';

// Fonts and Icons
import { FontAwesomeIcon } from "@fortawesome/react-fontawesome";
import { faHome, faCogs, faMicrochip, faDesktop, faChartBar, faBook} from "@fortawesome/free-solid-svg-icons";

export const Etude = () => {
  const activeChild = 'Etudes';

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

    </div>
  );
};
