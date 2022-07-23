import React from "react";
import styled from "styled-components";
import * as THREE from 'three';
import { VRButton } from 'three/examples/jsm/webxr/VRButton.js';

const Canvas = styled.canvas`
    background-color: #000;
    height: 100%;
    width: 100%;
`

const HiddenVideo = styled.video`
    display: none;
`


interface Props {
    className?: string,
    left?: MediaStream,
    right?: MediaStream,
}

export default class Viewport extends React.Component<Props> {
    private canvas = React.createRef<HTMLCanvasElement>();
    private left = React.createRef<HTMLVideoElement>();
    private right = React.createRef<HTMLVideoElement>();

    render(): React.ReactNode {
        return <div className={this.props.className}>
            <Canvas ref={this.canvas}/>
            <HiddenVideo autoPlay ref={this.left}/>
            <HiddenVideo autoPlay ref={this.right}/>
        </div> 
    }

    componentDidMount() {
        // Based on the assumption both Oculkust Quest 2 and Webcam have a diagonal feald of view of 90. But a diffrent 
        let fov = 0.830097;

        this.componentDidUpdate();
        const scene = new THREE.Scene();

        const camera = new THREE.OrthographicCamera( -1, 1, 1, -1, -1, 1  );
    
        const renderer = new THREE.WebGLRenderer({
            canvas: this.canvas.current!
        });
    
        renderer.setSize(3664, 1920, false); 
        renderer.xr.enabled = true;
        renderer.xr.cameraAutoUpdate = false;
        // @ts-ignore
        renderer.xr.getCamera = function() {
            return camera;
        }
    
        const textureLeft = new THREE.VideoTexture( this.left.current! );
        textureLeft.center = new THREE.Vector2( 0.5, 0.5 )
        textureLeft.rotation = 90  * (Math.PI/180);
    
        const left = new THREE.Mesh(
            new THREE.PlaneGeometry( 0.58952* fov, 2 * fov),
            new THREE.MeshBasicMaterial( { map: textureLeft } )
        );
        left.position.x = -0.5;
        scene.add( left );
    
        const textureRight= new THREE.VideoTexture( this.right.current! );
        textureRight.center = new THREE.Vector2( 0.5, 0.5 )
        textureRight.rotation = 90 * 3 * (Math.PI/180);
        const right = new THREE.Mesh(
            new THREE.PlaneGeometry( 0.58952 * fov, 2 * fov),
            new THREE.MeshBasicMaterial( { map: textureRight } )
        );
        right.position.x = 0.5;
        scene.add( right );
      
    
        renderer.setAnimationLoop( function () {
            renderer.render( scene, camera );
        } );
    
        document.body.appendChild( VRButton.createButton( renderer ) );
    }

    componentDidUpdate() {
        this.left.current!.srcObject = this.props.left ?? null;
        this.right.current!.srcObject = this.props.right ?? null;
    }
}