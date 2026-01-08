import matplotlib.pyplot as plt
import numpy as np

data = np.genfromtxt('KROG2_1.CSV', delimiter=',', names=True, dtype=float)

t_zel = data['T_zeljena']
t_tren = data['T_tren']
ventil = data['ventil']
cas = np.linspace(1, len(data['T_zeljena']), len(data['T_zeljena'])) 

plt.figure()
plt.plot(cas, t_tren, label="Trenutna temperatura")
plt.plot(cas, t_zel, label="Želena temperatura", linestyle='--')
plt.plot(cas, ventil, label="Položaj ventila", linestyle='dotted')
plt.xlabel("Čas [sekunde]")
plt.ylabel("Temperatura / položaj ventila")
plt.title("Pregled dnevnika ogrevanja")
plt.legend()
plt.grid(True)
plt.show()