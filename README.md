# MultiBodySimulation

**MultiBodySimulation** est une bibliothèque Python dédiée à la modélisation et à la simulation dynamique de systèmes mécaniques multi-corps en 3D.
Elle permet de simuler des systèmes articulés, des structures élastiques, des mécanismes en rotation/translation, et d'analyser leur comportement dynamique (fréquences propres, réponse temporelle, réponse fréquentielle, etc.).
Pour l'instant cette librairie est limitée aux **petits angles**.

---

## 📌 Fonctionnalités principales

- **Modélisation multi-corps** : Corps rigides, liaisons cinématiques, ressorts/amortisseurs linéaires et angulaires.
- **Simulation dynamique** : Intégration temporelle (ODE) avec gestion des conditions initiales et des excitations externes.
- **Analyse modale** : Calcul des fréquences propres et des modes de vibration.
- **Réponse fréquentielle** : Fonctions de transfert et analyse spectrale.
- **Validation croisée** : Comparaison avec des solutions analytiques ou issues d'autres solveurs (Scipy).
- **Visualisation** : Outils intégrés pour tracer les résultats (déplacements, vitesses, angles, etc.).

---

## 📦 Installation

### Prérequis
- Python ≥ 3.8
- NumPy, SciPy, Matplotlib

### Importation
```python
import socofer, sys
sys.path.append(socofer.lib_socofer_path)
# Corps de référence et corps libres
from MultiBodySimulation.MBSBody import MBSRigidBody3D, MBSReferenceBody3D
# Liaisons élastiques et liaisons mécaniques
from MultiBodySimulation.MBSMechanicalJoint import MBSLinkLinearSpringDamper, MBSLinkKinematic
# Système linéaire
from MultiBodySimulation.MBSMechanicalSystem import MBSLinearSystem
```
## 🚀 Exemples d'utilisation

### Système en torsion (3 masses) - temporel

Réponse temporelle d'un système de barres en torsion.

![System](examples/1-Exemples%20Simples%20-%20Petits%20Angles/Torsional_System.png)

```python
import numpy as np  
import matplotlib.pyplot as plt  
import sys,socofer  
sys.path.append(socofer.lib_socofer_path)  
from MultiBodySimulation.MBSBody import MBSRigidBody3D,MBSReferenceBody3D  
from MultiBodySimulation.MBSMechanicalJoint import MBSLinkLinearSpringDamper  
from MultiBodySimulation.MBSMechanicalSystem import MBSLinearSystem
  
J1 = 1.0  
J2 = 4.0  
J3 = 0.5  
kt1 = 10.0  
kt2 = 10.0  
kt3 = 4.0  
ct1 = kt1 * 8/100  
ct2 = kt2 * 8/100  
ct3 = kt3 * 8/100  
  
t_span = [0, 30]  
t_eval = np.linspace(t_span[0], t_span[1], 1000)  
dt = t_eval[1] - t_eval[0]  
  
# Positions  
x1 = 0.01  
x2 = x1 + 0.01  
x3 = x2 + 0.01  
  
# Torsion initiale  
theta_3_init = 20*np.pi/180  
y0_scipy = np.array([0.0, 0.0, theta_3_init, 0.0, 0.0, 0.0])  
  
f = 0.5032 # Fréquence de raisonnance de J1;k1  
theta0_sig = lambda t : np.sin(2*np.pi * f * t) * 20*np.pi/180 * 0  
omega0_sig = lambda t : 2*np.pi*f*np.cos(2*np.pi*f*t) * 20*np.pi/180 * 0
mecha_sys = MBSLinearSystem()  
mecha_sys.gravity = np.array([0.0,0.0,0.0])  
  
RefBody = MBSReferenceBody3D("Ref")  
RefBody.SetReferencePosition([0., 0., 0.])  
RefBody.SetRotationFunction(dtheta_x_func = theta0_sig)  
  
Mass1 = MBSRigidBody3D("Masse 1",  
                        mass = 1,  
                        inertia_tensor = J1)  
Mass1.SetReferencePosition([x1, 0., 0.])  
  
Mass2 = MBSRigidBody3D("Masse 2",  
                        mass = 1,  
                        inertia_tensor = J2)  
Mass2.SetReferencePosition([x2, 0., 0.])  
  
Mass3 = MBSRigidBody3D("Masse 3",  
                        mass = 1,  
                        inertia_tensor = J3)  
Mass3.SetReferencePosition([x3, 0., 0.])  
  
Mass3.ChangeInitialAngle([theta_3_init, 0., 0.])  
  
mecha_sys.AddRigidBody(RefBody)  
mecha_sys.AddRigidBody(Mass1)  
mecha_sys.AddRigidBody(Mass2)  
mecha_sys.AddRigidBody(Mass3)  
  
joint01 = MBSLinkLinearSpringDamper(RefBody,  
                                    [(x1)/2, 0., 0.],  
                                    Mass1,  
                                    [(x1)/2, 0., 0.],  
                                    angular_stiffness = kt1,  
                                    angular_damping = ct1)  
  
joint12 = MBSLinkLinearSpringDamper(Mass1,  
                                    [(x1+x2)/2, 0., 0.],  
                                    Mass2,  
                                    [(x1+x2)/2, 0., 0.],  
                                    angular_stiffness = kt2,  
                                    angular_damping = ct2)  
  
joint23 = MBSLinkLinearSpringDamper(Mass2,  
                                    [(x3+x2)/2, 0., 0.],  
                                    Mass3,  
                                    [(x3+x2)/2, 0., 0.],  
                                    angular_stiffness = kt3,  
                                    angular_damping = ct3)  
  
mecha_sys.AddLinkage(joint01)  
mecha_sys.AddLinkage(joint12)  
mecha_sys.AddLinkage(joint23)  
  
t_mbs, results = mecha_sys.RunDynamicSimulation(t_span, dt)  
  
theta1_mbs = results["Masse 1"].angles[0]  
theta2_mbs = results["Masse 2"].angles[0]  
theta3_mbs = results["Masse 3"].angles[0]  
  
## Affichage des solutions  
  
plt.figure()   
plt.plot(t_mbs, theta1_mbs * 180/np.pi, label=r"$\Theta_1$")  
plt.plot(t_mbs, theta2_mbs * 180/np.pi, label=r"$\Theta_2$")  
plt.plot(t_mbs, theta3_mbs * 180/np.pi, label=r"$\Theta_3$")  
plt.xlabel("Temps [s]")  
plt.ylabel(r"Angle $\theta_1$ [°]")  
plt.title(r"Barre torsion : $\theta_1$")  
plt.legend()  
plt.grid()  
plt.tight_layout()  
plt.show()
```

![Solution](examples/1-Exemples%20Simples%20-%20Petits%20Angles/arbre_torsion.png)


### Système bielle-ressort (2D) - temporel

Simule une bielle en rotation avec un ressort et un amortisseur.
```python
import numpy as np  
import matplotlib.pyplot as plt  
  
import sys, socofer  
sys.path.append(socofer.lib_socofer_path)  
  
from MultiBodySimulation.MBSBody import MBSRigidBody3D, MBSReferenceBody3D  
from MultiBodySimulation.MBSMechanicalJoint import (MBSLinkLinearSpringDamper,  
                                                     MBSLinkKinematic)  
from MultiBodySimulation.MBSMechanicalSystem import MBSLinearSystem  
  
# =============================================================================  
# PARAMÈTRES PHYSIQUES  
# =============================================================================  
m = 1.0 # masse de la bielle [kg]  
L = 0.5 # longueur de la bielle [m]  
J = m * L**2 / 12 # inertie au CDG (barre uniforme) [kg·m²]  
Jeq = J + (L/2)**2 * m  
  
k_ressort = 50.0 # raideur du ressort [N/m]  
c_ressort = 0.5 # amortissement du ressort [N·s/m]  
  
f = 0.5 # Hz  
theta_amp = 5 * np.pi / 180  
h_amp = L * np.sin(theta_amp)  
  
h_signal = lambda t: np.sin(2 * np.pi * f * t) * h_amp + np.sin(2 * np.pi * 1.5*f * t + np.pi/5) * h_amp / 3  
vh_signal = lambda t : 2 * np.pi * f * h_amp * np.cos(2 * np.pi * f * t) + \  
                    2 * np.pi * 3*f * np.cos(2 * np.pi * 1.5*f * t + np.pi/5) * h_amp / 3  
  
  
  
t_span = [0, 10.0]  
dt = 0.01  
t_eval = np.linspace(t_span[0], t_span[1], 1000)  
  
print(f"\n{'='*70}")  
print("BIELLE AVEC PIVOT ET RESSORT - Validation des couplages")  
print(f"{'='*70}")  
print(f"\nParamètres physiques:")  
print(f"  Masse bielle     : m = {m} kg")  
print(f"  Longueur         : L = {L} m")  
print(f"  Inertie au CDG   : J = {J:.6f} kg·m²")  
print(f"  Raideur ressort  : k = {k_ressort} N/m")  
print(f"  Amortissement    : c = {c_ressort} N·s/m")  
  
# =============================================================================  
# SOLUTION ANALYTIQUE (PETITS ANGLES)  
# =============================================================================  
print(f"\n{'='*70}")  
print("SOLUTION ANALYTIQUE")  
print(f"{'='*70}")  
  
# Équation du mouvement (petits angles) :  
# Quand la bielle tourne de θ, l'extrémité se déplace en Y de : y = L·sin(θ) ≈ L·θ  
# Force du ressort : F_y = -k·y = -k·L·θ  
# Moment au pivot : M_pivot = L·F_y = -k·L²·θ  
# Moment au CDG : M_CDG = (L/2)·F_y = -k·L²/2·θ  
#  
# Équation : Jeq·θ̈ + c·(L²/2)·θ̇ + k·(L²/2)·θ = 0  
# Jeq == J + (L/2)^2 * M

# =============================================================================  
# MODÉLISATION MBS  
# =============================================================================  
  
# Système MBS  
sys_mbs = MBSLinearSystem()  
sys_mbs.gravity = np.array([0., 0., 0.])  # Pas de gravité  
  
# Corps fixe (pivot)  
pivot_body = MBSReferenceBody3D("Pivot")  
pivot_body.SetReferencePosition([0., 0., 0.])  
  
# Corps fixe (reference)  
reference_body = MBSReferenceBody3D("Reference")  
reference_body.SetReferencePosition([L, 0., 0.])  
reference_body.SetDisplacementFunction(dy_func = h_signal)  
  
  
# Corps mobile (bielle)  
bielle = MBSRigidBody3D("Bielle", mass=m, inertia_tensor=J)  
bielle.SetReferencePosition([L/2, 0., 0.])  # CDG au milieu  
  
# Liaison pivot (bloque translations, bloque rotations X et Y, laisse Z libre)  
liaison_pivot = MBSLinkKinematic(  
    pivot_body, [0., 0., 0.],  
    bielle, [0., 0., 0.],  
    Tx=1, Ty=1, Tz=1,  # Translations bloquées  
  Rx=1, Ry=1,        # Rotations X, Y bloquées  
  Rz=0,              # Rotation Z libre (autour de Z)  
)  
  
# Ressort à l'extrémité (position de référence [L, 0, 0])  
# En petits angles, le déplacement Y de l'extrémité ≈ L·θ  
ressort = MBSLinkLinearSpringDamper(  
    reference_body, [L, 0., 0.],  
    bielle, [L, 0., 0.],  
    stiffness=[0., k_ressort, 0.],  # Ressort en Y seulement  
  damping=[0., c_ressort, 0.]  
)  
  
# Assemblage  
sys_mbs.AddRigidBody(pivot_body)  
sys_mbs.AddRigidBody(reference_body)  
sys_mbs.AddRigidBody(bielle)  
sys_mbs.AddLinkage(liaison_pivot)  
sys_mbs.AddLinkage(ressort)  
  
sys_mbs.AssemblyMatrixSystem(print_report=True)  
  
  
# =============================================================================  
# SIMULATION  
# =============================================================================   
  
import time  
t_start = time.time()  
t_mbs, results = sys_mbs.RunDynamicSimulation(  
    t_span=t_span,  
    dt=dt,  
    ode_method="BDF",  
    max_angle_threshold=15.0 # Alerte si > 15°  
)  
t_end = time.time()  
  
print(f"\nTemps de calcul : {t_end - t_start:.3f} s")  
  
# Extraction des résultats  
bielle_results = results["Bielle"]  
theta_mbs = bielle_results.angles[2]  
# Position de l'extrémité (reconstruction)  
ext_mbs = bielle_results.get_connected_point_motion([L,0,0])  
y_ext_mbs = ext_mbs.positions[1]  
x_ext_mbs = ext_mbs.positions[0]
```

![Solution](examples/1-Exemples%20Simples%20-%20Petits%20Angles/bielle_reaction.png)

### Analyse modale - système en torsion

```python
mecha_sys = ... # Système mécanique exemple précédent 
natural_freq = mecha_sys.ComputeNaturalFrequencies(sort_values=True,  
                                                    drop_zeros=True)  
print("Fréquences propres du système (MBS) : ")  
for f in natural_freq :  
    print(f">> f = {f:.5e} Hz")  
  
  
  
mbs_modal_result = mecha_sys.ComputeModalAnalysis(sort_values = True,  
                                                            drop_zeros = True,)  
# Fréquences propres : liste des fréquences propres
freq_vector_mbs = mbs_modal_result.GetNaturalFrequencies()
# Fréquences propres du système (MBS) : 
>> f = 1.61801e-01 Hz
>> f = 4.78133e-01 Hz
>> f = 7.36962e-01 Hz

# Analyse modale
# modes[body_name] = [[x, y, z, rx, ry, rz], # mode 1  
# 				 	  [x, y, z, rx, ry, rz], # mode 2 ...
modal_result_dict = mbs_modal_result.GetDisplacementsByBodies()  
```


![Modal](examples/2-Exemples%20-%20etude%20frequentielle/analyse_modale.png)

### Analyse réponse fréquentielle - système en torsion
```python
mecha_sys = ... # Système mécanique exemple précédent 

# theta_z --> axe n°3 (x : 0, y : 1, z : 2, theta_z : 3, ... )
freqres = mecha_sys.ComputeFrequencyDomainResponse([  
            ("Ref", 3, "Masse 1", 3), 
            ("Ref", 3, "Masse 2", 3),  
            ("Ref", 3, "Masse 3", 3),  
                        ])  
                        
# On sélectionne toutes les fonctions de transfert calculées
G = freqres.SelectTransferFunctionObject_byLocId(None)

plt.figure(figsize=(7,8))  
plt.subplot(311)  
plt.loglog(G.frequency, G.module, label = G.names)  
for w0 in freqres.GetNaturalFrequencies() :  
    plt.axvline(w0, color = "grey")  
plt.title("Réponse 'Amplitude'")  
plt.ylabel(r"$|H(\omega)|$")  
plt.legend()  
plt.grid(True)  
  
plt.subplot(312)  
plt.title("Réponse 'Phase'")  
plt.semilogx(G.frequency, G.phase, label = G.names)  
for w0 in freqres.GetNaturalFrequencies() :  
    plt.axvline(w0, color = "grey")  
plt.ylabel(r"$\phi_(H) (\omega)$ [°]")  
plt.grid(True)  
  
plt.subplot(313)  
plt.title("Réponse 'Power Spectral Density'")  
plt.loglog(G.frequency, G.powerSpectralDensity, label = G.names)  
for w0 in freqres.GetNaturalFrequencies() :  
    plt.axvline(w0, color = "grey")  
plt.ylabel(r"$PSD_(H) (\omega)|$")  
plt.grid(True)  
  
plt.xlabel("Freq (Hz)")  
plt.tight_layout()
```

![Modal](examples/2-Exemples%20-%20etude%20frequentielle/freq_response.png)
