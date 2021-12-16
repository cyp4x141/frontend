import * as THREE from 'three';

import { loadTexture } from 'utils/models';
import gridGround from 'assets/images/ground.png';

export default class Ground {
  constructor() {
    this.type = 'default';
    this.loadedMap = null;
    this.updateMap = null;
    this.mesh = null;
    this.geometry = null;
    this.initialized = false;
    this.inNaviMode = null;
    this.showCameraView = false;

    loadTexture(gridGround, (texture) => {
      this.geometry = new THREE.PlaneGeometry(1, 1);
      this.mesh = new THREE.Mesh(
        this.geometry,
        new THREE.MeshBasicMaterial({ map: texture }),
      );
    });
  }

  initialize(coordinates) {
    if (!this.mesh) {
      return false;
    }

    if (this.loadedMap === this.updateMap && !this.render(coordinates)) {
      return false;
    }
    this.initialized = true;
    return true;
  }

  loadGrid(coordinates) {
    loadTexture(gridGround, (texture) => {
      console.log('using grid as ground image...');
      this.mesh.material.map = texture;
      this.mesh.type = 'grid';
      this.mesh.visible = true;
      this.render(coordinates);
    });
  }

  update(x, y, coordinates) {
    if (this.initialized !== true) {
      return;
    }
    this.loadGrid(coordinates);
    const position = coordinates.applyOffset({ x: x, y: y });
    this.mesh.position.set(position.x, position.y, 0);
  
  }

  updateImage(mapName) {
    this.updateMap = mapName;
  }

  render(coordinates, mapName = 'defaults') {
    console.log('rendering ground image...');
    const xres = 8192;
    const yres = 8192;
    const mpp = 0.125;
    const xorigin = 4096;
    const yorigin = 4096;
    let position = coordinates.applyOffset({ x: xorigin, y: yorigin });
    if (position === null) {
      console.warn('Cannot find position for ground mesh!');
      return false;
    }
    // NOTE: Setting the position to (0, 0) makes the center of
    // the ground image to overlap with the offset point, which
    // is the car position on the first received frame.
    position = { x: 0, y: 0 };

    this.mesh.position.set(position.x, position.y, - 0.5);
    this.mesh.scale.set(xres * mpp, yres * mpp, 1);
    this.mesh.material.needsUpdate = true;
    this.mesh.overdraw = false;

    return true;
  }

  titleCaseToSnakeCase(str) {
    return str.replace(/\s/g, '_').toLowerCase();
  }
}
