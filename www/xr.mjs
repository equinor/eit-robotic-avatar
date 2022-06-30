import * as THREE from 'three';
import Cameras from "./modules/cameras.mjs";

async function main() {
    const scene = new THREE.Scene();
    const height = window.innerHeight;
    const width = window.innerWidth;

    const camera = new THREE.OrthographicCamera( -1, 1, 1, -1, -1, 1  );

    const renderer = new THREE.WebGLRenderer();
    renderer.setSize( window.innerWidth, window.innerHeight );
    document.body.appendChild( renderer.domElement );

    const cameras = new Cameras(document.getElementById("leftVideo"), document.getElementById("rightVideo"));
    await cameras.loadFromUser();

    const textureLeft = new THREE.VideoTexture( cameras.left );
    textureLeft.center = new THREE.Vector2( 0.5, 0.5 )
    textureLeft.rotation = 90 * 3 * (Math.PI/180);

    const left = new THREE.Mesh(
        new THREE.PlaneGeometry( 1, 2 ),
        new THREE.MeshBasicMaterial( { map: textureLeft } )
    );
    left.position.x = -0.5;
    scene.add( left );

    const textureRight= new THREE.VideoTexture( cameras.right );
    textureRight.center = new THREE.Vector2( 0.5, 0.5 )
    textureRight.rotation = 90 * (Math.PI/180);
    const right = new THREE.Mesh(
        new THREE.PlaneGeometry( 1, 2 ),
        new THREE.MeshBasicMaterial( { map: textureRight } )
    );
    right.position.x = 0.5;
    scene.add( right );

    function animate() {
        requestAnimationFrame( animate );
        renderer.render( scene, camera );
    }
    animate();
}

async function xrmain() {
    let xr = navigator.xr;

    let session = await xr.requestSession("immersive-vr");

    let ref_frame = await session.requestReferenceSpace("local");

    function vrframe(time, frame) {
        console.log(frame.getViewerPose(ref_frame).transform.orientation.toJSON());

       renderer.render( scene, camera );
        session.requestAnimationFrame(vrframe);
    }
    
    window.vrtest = session;
    window.vr_frame = vrframe;

    
    let glCanvas = document.getElementById("glcanvas");
    let gl = glCanvas.getContext("webgl", { xrCompatible: true });

    session.updateRenderState({
        baseLayer: new XRWebGLLayer(session, gl)
    });

    session.requestAnimationFrame(vrframe);
}

document.getElementById("vr").onclick = main;