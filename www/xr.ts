import * as THREE from 'three';
import Cameras from "./modules/cameras";

import { fromOffers } from "./modules/rtc";
import { pullOffers, postAnswer } from "./modules/server";

import { VRButton } from 'three/examples/jsm/webxr/VRButton.js';

async function main() {
    try {
        document.getElementById("start")!.hidden = true;
        let offers = await pullOffers();
        console.log(offers);
        let con = await fromOffers(offers);
        let answer = await con.createAnswers();
        console.log(answer);
        await postAnswer(answer);
        let streams = con.getStreams();
        const cameras = new Cameras(document.getElementById("leftVideo")! as HTMLVideoElement, document.getElementById("rightVideo")! as HTMLVideoElement);
        cameras.setStreams(streams)
        await setup3D(cameras);
    } catch (err) {
        console.log(err.name + ": " + err.message);
    }
}

async function setup3D(cameras) {
    const scene = new THREE.Scene();

    const camera = new THREE.OrthographicCamera( -1, 1, 1, -1, -1, 1  );

    const renderer = new THREE.WebGLRenderer({
        canvas: document.getElementById("view")! as HTMLCanvasElement
    });

    renderer.setSize(3664, 1920, false); 
    renderer.xr.enabled = true;
    renderer.xr.cameraAutoUpdate = false;
    // @ts-ignore
    renderer.xr.getCamera = function() {
        return camera;
    }

    document.body.appendChild( renderer.domElement );

    const textureLeft = new THREE.VideoTexture( cameras.left );
    textureLeft.center = new THREE.Vector2( 0.5, 0.5 )
    textureLeft.rotation = 90  * (Math.PI/180);

    const left = new THREE.Mesh(
        new THREE.PlaneGeometry( 1, 2 ),
        new THREE.MeshBasicMaterial( { map: textureLeft } )
    );
    left.position.x = -0.5;
    scene.add( left );

    const textureRight= new THREE.VideoTexture( cameras.right );
    textureRight.center = new THREE.Vector2( 0.5, 0.5 )
    textureRight.rotation = 90 * 3 * (Math.PI/180);
    const right = new THREE.Mesh(
        new THREE.PlaneGeometry( 1, 2 ),
        new THREE.MeshBasicMaterial( { map: textureRight } )
    );
    right.position.x = 0.5;
    scene.add( right );
  

    renderer.setAnimationLoop( function () {
        renderer.render( scene, camera );
    } );

    document.body.appendChild( VRButton.createButton( renderer ) );

}

document.getElementById("VR")!.hidden = true;
document.getElementById("start")!.onclick = main;

