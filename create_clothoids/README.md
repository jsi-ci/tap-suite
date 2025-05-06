# Clothoids

This package enables to create multiple sequential asymmetric clothoid spline from points.

This module has been developed within the project "Multiobjective discovery of driving strategies for autonomous vehicles", which was financially supported by the Slovenian Research Agency under the funding No. Z2-7581, and by NERVteh, raziskave in razvoj, d.o.o.

References:
- D.J. Walton, D.S. Meek, "A controlled clothoid spline", Computers & Graphics 29 (2005) 353-363
- http://www.lara.prd.fr/_media/users/franciscogarcia/a_controlled_clothoid_spline.pdf

### Installation 

```sh
pip install --no-index --find-links=https://repo.ijs.si/erikdovgan/clothoids/raw/master/dist/clothoids-0.0.10-py3-none-any.whl create_clothoids
```

### Execution Example

```sh
import matplotlib.pyplot as plt
import numpy as np
from create_clothoids import create_clothoids

route_center=create_clothoids.create_clothoid(np.array([[2.4,5.2],[5.4,6.9],[6.8,9.5],[8.1,10.2]]))
plt.plot(route_center[:,0], route_center[:,1])
plt.show()

route_center=create_clothoids.create_clothoid_from_mandatory_points_with_angles(np.array([[2.4,5.2, 0.2],[5.1,7.5, 0.5],[8.1,10.2, 0.4]]))
plt.plot(route_center[:,0], route_center[:,1])
plt.show()

```

### Recompilation
```sh
python setup.py sdist bdist_wheel
```