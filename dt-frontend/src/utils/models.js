import * as THREE from 'three';
import { OBJLoader } from 'three/examples/jsm/loaders/OBJLoader';
import { MTLLoader } from 'three/examples/jsm/loaders/MTLLoader';

// The two loaders for material and object files, respectively.
const mtlLoader = new MTLLoader();
const textureLoader = new THREE.TextureLoader();

THREE.TextureLoader.prototype.crossOrigin = '';

const loadedMaterialAndObject = {};

export function loadObject(materialFile, objectFile, scale, callback) {
  function placeMtlAndObj(loaded) {
    if (callback) {
      const object = loaded.clone();
      callback(object);
    }
  }

  if (loadedMaterialAndObject[objectFile]) {
    placeMtlAndObj(loadedMaterialAndObject[objectFile]);
  } else {
    new Promise((resolve, reject) => {
      if (materialFile) {
        mtlLoader.load(materialFile, (materials) => {
          materials.preload();
          resolve(materials);
        });
      } else {
        resolve(null);
      }
    }).then((materials) => {
      const objLoader = new OBJLoader();
      if (materials) {
        objLoader.setMaterials(materials);
      }

      objLoader.load(objectFile, (loaded) => {
        loaded.name = objectFile;
        loaded.scale.set(scale.x, scale.y, scale.z);
        loadedMaterialAndObject[objectFile] = loaded;
        placeMtlAndObj(loaded);
      });
    }).catch(() => {
      console.error('Failed to load object.');
    });
  }
}

export function loadTexture(textureFile, onLoadCallback, onErrorCallback) {
  textureLoader.load(textureFile, onLoadCallback, undefined, onErrorCallback);
}

export function loadMaterial(materialFile, onLoadCallback) {
  mtlLoader.load(materialFile, onLoadCallback);
}
