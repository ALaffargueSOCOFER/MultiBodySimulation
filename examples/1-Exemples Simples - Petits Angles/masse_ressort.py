

import numpy as np
import matplotlib.pyplot as plt
from scipy.integrate import solve_ivp

import sys, socofer
sys.path.append(socofer.lib_socofer_path)

from MultiBodySimulation.MBSBody import MBSRigidBody3D,MBSReferenceBody3D
from MultiBodySimulation.MBSMechanicalJoint import MBSLinkLinearSpringDamper
from MultiBodySimulation.MBSMechanicalSystem import MBSLinearSystem

M = 1.0
J = 1
k = 20
c = 1.5

zref = 0.0
zinit = 0.1
t_span = [0,8]
t_eval = np.linspace(t_span[0], t_span[1], 500)
dt = t_eval[1]-t_span[0]

# Solving with scipy
Amatrix = np.array([[0,1],
           [-k/M,-c/M]])
bvec = np.array([0.,-9.81])

r = solve_ivp(lambda t,z : Amatrix.dot(z) + bvec,
               t_span,
               [zinit,0.],
               t_eval=t_eval )
z_scipy = r.y[0]


ref_position = np.array([0,0,zref])
initial_body_position = np.array([0,0,zinit])
g_vector = np.array([0,0,-9.81])

# Solving with multibody tools
# Création du système
mecha_sys = MBSLinearSystem()
# Ajouter la gravité -9.81 en z
mecha_sys.gravity = g_vector
# Corps de référence = immobile
refBody = MBSReferenceBody3D("Ground")
# Corps simulé
massBody = MBSRigidBody3D("Mass",M,J)
# Placer les corps dans le repère global
massBody.SetReferencePosition(ref_position)
# Créer le ressort-amortisseur
joint = MBSLinkLinearSpringDamper(refBody, # Corps 1
            ref_position, #Point d'attache au corps 1 dans le repère global
            massBody, # Corps 2
            ref_position, #Point d'attache au corps 2 dans le repère global
            k, # Raideur
            c,) # Amortissement
# Assembler le système
mecha_sys.AddRigidBody(refBody)
mecha_sys.AddRigidBody(massBody)
mecha_sys.AddLinkage(joint)
# On repositionne le corps dans le repère global pour initialiser la simulation
massBody.ChangeInitialPosition(initial_body_position)
#Simulation
t_mbs, results = mecha_sys.RunDynamicSimulation(t_span=t_span,dt=dt)
#Récupérer les résultats
z_result = results["Mass"].positions[2] # Indice 2 = direction Z

plt.figure()
plt.plot(t_eval, z_scipy, label = "Scipy", color = "blue", lw=2)
plt.plot(t_mbs, z_result, ls="--", label = "MultiBody", color = "darkorange", lw=2)
plt.grid(True)
plt.xlabel('time (s)')
plt.ylabel("Position [m]")
plt.title("Comparaison : masse ressort simple")
plt.legend()

plt.show()