import * as THREE from 'three';

import Coordinates from 'renderer/coordinates';
import AutoDrivingCar from 'renderer/adc';
import Ground from 'renderer/ground';

class Renderer {
  constructor() {
    // Disable antialias for mobile devices.

    this.coordinates = new Coordinates();
    this.renderer = new THREE.WebGLRenderer({
      // Transparent background
      alpha: true,
    });
    this.scene = new THREE.Scene();
    this.scene.background = new THREE.Color(0x000C17);

    // The dimension of the scene
    this.dimension = {
      width: 0,
      height: 0,
    };

    // The ground.
    this.ground =  new Ground();

    // The main autonomous driving car.
    this.adc = new AutoDrivingCar('adc', this.scene);
    // this.coordinates.initialize(0, 0);

  }

  initialize(canvasId, width, height) {
    this.canvasId = canvasId;

    // Camera
    this.viewAngle = 0.8;
    this.viewDistance = 20;
    this.camera = new THREE.PerspectiveCamera(
      60,
      width / height,
      1,
      200,
    );
    this.camera.name = 'camera';
    this.scene.add(this.camera);
    this.updateDimension(width, height);
    this.renderer.setPixelRatio(window.devicePixelRatio);
    const container = document.getElementById(canvasId);
    container.appendChild(this.renderer.domElement);
    const ambient = new THREE.AmbientLight(0x444444);
    const directionalLight = new THREE.DirectionalLight(0xffeedd);
    directionalLight.position.set(0, 0, 1).normalize();

    // The orbit axis of the OrbitControl depends on camera's up vector
    // and can only be set during creation of the controls. Thus,
    // setting camera up here. Note: it's okay if the camera.up doesn't
    // match the point of view setting, the value will be adjusted during
    // each update cycle.
    this.camera.up.set(0, 0, 1);

    this.scene.add(ambient);
    this.scene.add(directionalLight);

    // Actually start the animation.
    this.maybeInitializeOffest(0, 0);
    this.animate();
  }

  updateHdmp(markers) {
    const material = new THREE.LineBasicMaterial({
      color: 0xffffff
    });

    markers.forEach((marker) => {
      const points = marker.points;
      const roadPoints = [];
      points.forEach((point) => {
        roadPoints.push(new THREE.Vector3( point.x, point.y, 0));
      })
      const geometry = new THREE.BufferGeometry().setFromPoints( roadPoints );
      const line = new THREE.LineSegments( geometry, material );
      this.scene.add( line );
    })
  }

  maybeInitializeOffest(x, y, forced_update = false) {
    if (!this.coordinates.isInitialized() || forced_update) {
      console.log("init offset")
      this.coordinates.initialize(x, y);
    }
  }

  updateDimension(width, height) {

    this.camera.aspect = width / height;
    this.camera.updateProjectionMatrix();
    this.renderer.setSize(width, height);

    this.dimension.width = width;
    this.dimension.height = height;
  }

  adjustCamera(target) {
    this.camera.fov = 60;
    this.camera.near = 1;
    this.camera.far = 200;
    let deltaX = (this.viewDistance * Math.cos(target.rotation.y)
            * Math.cos(this.viewAngle));
    let deltaY = (this.viewDistance * Math.sin(target.rotation.y)
            * Math.cos(this.viewAngle));
    let deltaZ = this.viewDistance * Math.sin(this.viewAngle);

    this.camera.position.set( target.position.x - deltaX, target.position.y - deltaY, target.position.z + deltaZ );
    this.camera.up.set(0, 0, 1);
    this.camera.lookAt(target.position.x + deltaX, target.position.y + deltaY, 0);
    this.camera.updateProjectionMatrix();
  }

  // Render one frame. This supports the main draw/render loop.
  render() {
    // TODO should also return when no need to update.
    if (!this.coordinates.isInitialized()) {
      return;
    }

    // Return if the car mesh is not loaded yet, or the ground is not
    // loaded yet.
    if (!this.adc.mesh || !this.ground.mesh) {
      return;
    }

    // Upon the first time in render() it sees ground mesh loaded,
    // added it to the scene.
    if (this.ground.type === 'default' && !this.ground.initialized) {
      this.ground.initialize(this.coordinates);
      this.ground.mesh.name = 'ground';
      this.scene.add(this.ground.mesh);
    }

    this.adjustCamera(this.adc.mesh);
    this.renderer.render(this.scene, this.camera);
  }

  animate() {
    requestAnimationFrame(() => {
      this.animate();
    });

    this.render();
  }

  updatePose(x, y, z) {
    this.adc.update(this.coordinates, x, y, z);
  }
}

const RENDERER = new Renderer();

export default RENDERER;
