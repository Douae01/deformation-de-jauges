// src/App.js
import "./App.css";
import React from "react";
import { BrowserRouter as Router, Routes, Route } from "react-router-dom";

// Router pages
import { Home } from "./pages/Home";
import { Circuit } from "./pages/Circuit";
import { Stm } from "./pages/Stm";
import { Ihm } from "./pages/Ihm";
import { Etude } from "./pages/Etude";
import { Biblio } from "./pages/Biblio";

function App() {
  return (
    <Router>
      <Routes>
        <Route path="/" element={<Home />} />
        <Route path="/Circuit" element={<Circuit />} />
        <Route path="/Stm" element={<Stm />} />
        <Route path="/Ihm" element={<Ihm />} />
        <Route path="/Etude" element={<Etude />} />
        <Route path="/Biblio" element={<Biblio />} />
      </Routes>
    </Router>
  );
}

export default App;
