import React, { useState } from "react";
import LeftSidebar from "../components/LeftSidebar";
import Header from "../components/Header";
import {
  OverviewIcon,
  IntegrationIcon,
  PlansIcon,
  TransactionsIcon,
  SettingsIcon,
  ContactIcon,
  initialProfileState,
} from "../helpers/profileConstants";

export function Profile() {
  // Initialisation des données du profil avec un état initial
  const [profileData, setProfileData] = useState(initialProfileState);

  // Fonction pour gérer les changements d'état pour chaque champ du profil
  const handleChange = (field, value) => {
    setProfileData((prevData) => ({
      ...prevData,
      [field]: value,
    }));
  };

  // Fonction pour sauvegarder les modifications du profil
  const Enregistrer = (e) => {
    e.preventDefault();

    // Vérification de la confirmation du mot de passe
    if (profileData.newPassword !== profileData.confirmPassword) {
      alert("Les nouveaux mots de passe ne correspondent pas");
      return;
    }

    // Mise à jour des données du profil et réinitialisation des champs de mot de passe
    setProfileData((prevData) => ({
      ...prevData,
      phone: profileData.tempPhone || prevData.phone,
      address: profileData.tempAddress || prevData.address,
      postal: profileData.tempPostal || prevData.postal,
      city: profileData.tempCity || prevData.city,
      country: profileData.tempCountry || prevData.country,
      success: true,
      newPassword: "",
      confirmPassword: "",
    }));

    // Affichage temporaire du message de succès
    setTimeout(() => {
      setProfileData((prevData) => ({
        ...prevData,
        success: false,
      }));
    }, 1500);
  };

  return (
    <div className="flex h-screen">
      {/* Barre latérale gauche */}
      <div className="w-1/6 flex-shrink-0">
        <LeftSidebar activeChild={null}>
        <div href="/" name="Accueil" icon={<OverviewIcon />} />
          <div href="/Circuit" name="Circuit" icon={<IntegrationIcon />} />
          <div href="/Stm" name="STM32" icon={<PlansIcon />} />
          <div href="/Ihm" name="IHM" icon={<TransactionsIcon />} />
          <div href="/Etude" name="Etudes" icon={<SettingsIcon />} />
          <div href="/Biblio" name="Bibliographie" icon={<ContactIcon />} />
        </LeftSidebar>
      </div>

      {/* Zone de contenu principal */}
      <div className="flex flex-col p-[10px] w-[1200px] ml-[70px]">
        <Header name="CHOUBRI" />

        <div className="flex-1 p-2 w-[900px] ml-[100px]">
          {/* Affichage du message de succès lors de la sauvegarde des données */}
          {profileData.success && (
            <div className="bg-green-500 text-white p-2 rounded-md mb-4">
              Vos informations sont bien enregistrées !
            </div>
          )}

          {/* Formulaire de profil */}
          <div className="bg-white p-5 rounded-lg shadow-sm">
            <h3 className="text-indigo-600 text-lg mb-2 font-bold">
              Vos informations
            </h3>
            <div className="flex flex-col justify-center gap-3">
              {/* Affichage du nom et prénom de l'utilisateur */}
              <div className="flex justify-center gap-20 align-items-start">
                <div className="flex flex-col flex-1">
                  <span className="mb-0.5 text-black text-base font-semibold">
                    Nom
                  </span>
                  <span className="bg-gray-200 p-2 rounded-lg text-gray-600">
                    CHOUBRI
                  </span>
                </div>
                <div className="flex flex-col flex-1">
                  <span className="mb-0.5 text-black text-base font-semibold">
                    Prénom
                  </span>
                  <span className="bg-gray-200 p-2 rounded-lg text-gray-600">
                    Mohamed
                  </span>
                </div>
              </div>
              {/* Affichage de la nationalité et du numéro de téléphone */}
              <div className="flex justify-center gap-20 align-items-start">
                <div className="flex flex-col flex-1">
                  <span className="mb-0.5 text-black text-base font-semibold">
                    Nationalité
                  </span>
                  <span className="bg-gray-200 p-2 rounded-lg text-gray-600">
                    {profileData.country}
                  </span>
                </div>
                <div className="flex flex-col flex-1">
                  <span className="mb-0.5 text-black text-base font-semibold">
                    Numéro de Téléphone
                  </span>
                  <span className="bg-gray-200 p-2 rounded-lg text-gray-600">
                    {profileData.phone}
                  </span>
                </div>
              </div>
              {/* Affichage de l'email et de l'adresse */}
              <div className="flex justify-center gap-20 align-items-start">
                <div className="flex flex-col flex-1">
                  <span className="mb-0.5 text-black text-base font-semibold">
                    E-mail
                  </span>
                  <span className="bg-gray-200 p-2 rounded-lg text-gray-600">
                    m3eljily@enib.fr
                  </span>
                </div>
                <div className="flex flex-col flex-1">
                  <span className="mb-0.5 text-black text-base font-semibold">
                    Adresse
                  </span>
                  <span className="bg-gray-200 p-2 rounded-lg text-gray-600">
                    {profileData.address}
                  </span>
                </div>
              </div>
            </div>
          </div>

          <div className="bg-white p-4 rounded-lg shadow-sm mt-5">
            <h3 className="text-indigo-600 text-lg mb-2 font-bold">
              Modifiez Votre Profil
            </h3>
            {/* Formulaire de modification de profil */}
            <form onSubmit={Enregistrer}>
              <div className="flex flex-col gap-6">
                <div className="flex gap-6">
                  <div className="flex flex-col flex-1">
                    <label htmlFor="address" className="mb-0.5 text-black text-base font-semibold">
                      Adresse
                    </label>
                    <input
                      type="text"
                      id="address"
                      placeholder="Kingston"
                      className="p-2 border border-gray-300 bg-gray-200 rounded-lg text-gray-600"
                      onChange={(e) => handleChange("tempAddress", e.target.value)}
                    />
                  </div>
                </div>
                {/* Autres champs : code postal, ville, pays, numéro de téléphone */}
                <div className="flex gap-6">
                  <div className="flex flex-col flex-1">
                    <label htmlFor="postal" className="mb-0.5 text-black text-base font-semibold">
                      Code Postal
                    </label>
                    <input
                      type="text"
                      id="postal"
                      placeholder="5236"
                      className="p-2 border border-gray-300 bg-gray-200 rounded-lg text-gray-600"
                      onChange={(e) => handleChange("tempPostal", e.target.value)}
                    />
                  </div>
                  <div className="flex flex-col flex-1">
                    <label htmlFor="city" className="mb-0.5 text-black text-base font-semibold">
                      Ville
                    </label>
                    <input
                      type="text"
                      id="city"
                      placeholder="Kingston"
                      className="p-2 border border-gray-300 bg-gray-200 rounded-lg text-gray-600"
                      onChange={(e) => handleChange("tempCity", e.target.value)}
                    />
                  </div>
                </div>
                <div className="flex gap-6">
                  <div className="flex flex-col flex-1">
                    <label htmlFor="country" className="mb-0.5 text-black text-base font-semibold">
                      Pays
                    </label>
                    <input
                      type="text"
                      id="country"
                      placeholder="France"
                      className="p-2 border border-gray-300 bg-gray-200 rounded-lg text-gray-600"
                      onChange={(e) => handleChange("tempCountry", e.target.value)}
                    />
                  </div>
                  <div className="flex flex-col flex-1">
                    <label htmlFor="phone" className="mb-0.5 text-black text-base font-semibold">
                      Numéro de Téléphone
                    </label>
                    <input
                      type="tel"
                      id="phone"
                      placeholder="+33 *********"
                      className="p-2 border border-gray-300 bg-gray-200 rounded-lg text-gray-600"
                      onChange={(e) => handleChange("tempPhone", e.target.value)}
                    />
                  </div>
                </div>
              </div>

              {/* Section pour modifier le mot de passe */}
              <h4 className="text-indigo-600 text-lg mb-2 mt-4 font-bold">Modification du Mot de passe</h4>
              <div>
                <div className="flex flex-col flex-1 mb-1.5">
                  <input
                    type="password"
                    id="current-password"
                    placeholder="Mot de passe actuel"
                    className="p-2 border border-gray-300 bg-gray-200 rounded-lg text-gray-600"
                    onChange={(e) => handleChange("currentPassword", e.target.value)}
                  />
                </div>
                <div className="flex flex-col flex-1 mb-1.5">
                  <input
                    type="password"
                    id="new-password"
                    placeholder="Nouveau mot de passe"
                    className="p-2 border border-gray-300 bg-gray-200 rounded-lg text-gray-600"
                    onChange={(e) => handleChange("newPassword", e.target.value)}
                  />
                </div>
                <div className="flex flex-col flex-1 mb-0">
                  <input
                    type="password"
                    id="confirm-password"
                    placeholder="Confirmer le nouveau mot de passe"
                    className="p-2 border border-gray-300 bg-gray-200 rounded-lg text-gray-600"
                    onChange={(e) => handleChange("confirmPassword", e.target.value)}
                  />
                </div>
                <br />
              </div>

              <div className="flex justify-end gap-4">
                {/* Bouton pour annuler les modifications */}
                <button type="reset" className="px-4 py-2 rounded-md text-sm cursor-pointer bg-gray-200 text-gray-600">Annuler</button>
                {/* Bouton pour enregistrer les modifications */}
                <button type="submit" className="rounded-md bg-indigo-600 px-3.5 py-2.5 text-center text-sm font-semibold text-white shadow-sm hover:bg-indigo-500 focus-visible:outline focus-visible:outline-2 focus-visible:outline-offset-2 focus-visible:outline-indigo-600" onClick={Enregistrer}>Enregistrer</button>
              </div>
            </form>
          </div>
        </div>
      </div>
    </div>
  );
}

export default Profile;
