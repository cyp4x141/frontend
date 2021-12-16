import _ from 'lodash';

import carMaterial from 'assets/models/car.mtl';
import carObject from 'assets/models/car.obj';
import { loadObject } from 'utils/models';

const CAR_PROPERTIES = {
  adc: {
    menuOptionName: 'showPositionLocalization',
    carMaterial,
  },
  planningAdc: {
    menuOptionName: 'showPlanningCar',
    carMaterial: null,
  },
  shadowAdc: {
    menuOptionName: 'showPositionShadow',
    carMaterial: null,
  },
};


export default class AutoDrivingCar {
  constructor(name, scene) {
    this.mesh = null;
    this.name = name;

    const properties = CAR_PROPERTIES[name];
    if (!properties) {
      console.error('Car properties not found for car:', name);
      return;
    }

    // NOTE: loadObject takes some time to update this.mesh.
    // This call is asynchronous.
    loadObject(properties.carMaterial, carObject, { x: 1, y: 1, z: 1 }, (object) => {
      this.mesh = object;
      this.mesh.rotation.x = Math.PI / 2;
      this.mesh.visible = false;
      scene.add(this.mesh);
    });
  }

  update(coordinates, x, y, z) {
    if (!this.mesh || !_.isNumber(x) || !_.isNumber(y)) {
      return;
    }

    this.mesh.visible = true;
    const position = coordinates.applyOffset({ x: x, y: y });
    if (position === null) {
      return;
    }
    this.mesh.position.set(position.x, position.y, 0);
    this.mesh.rotation.y = z;
  }

  resizeCarScale(x, y, z) {
    if (!this.mesh) {
      return;
    }
    this.mesh.scale.set(x, y, z);
  }
}
