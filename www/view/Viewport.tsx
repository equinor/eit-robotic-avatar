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
    onTrack?: (track: Tracking) => void,
}

interface Tracking {
    rx: number,
    ry: number,
    rz: number,
    l: Controller,
    r: Controller,
}

interface Controller {
    x: number, // Thumb Sticks X
    y: number, // Thumb Sticks X
    a: boolean, // A or X button
    b: boolean, // B or Y button
    c: number, // Trigger
    d: number, // Grip
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
        // Based on the assumption both Oculus Quest 2 and Webcam have a diagonal field of view of 90. But a different aspect ratio.
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
      
    
        renderer.setAnimationLoop( (_, xrframe) => {
            if(renderer.xr.isPresenting && this.props.onTrack) {
                const r = xrframe.getViewerPose(renderer.xr.getReferenceSpace()!)!.transform.orientation;
                let track = {
                    rx: r.x,
                    ry: r.y,
                    rz: r.z,
                    l: {x: 0,y: 0, a: false,b: false,c: 0,d: 0},
                    r: {x: 0,y: 0, a: false,b: false,c: 0,d: 0},
                }

                xrframe.session.inputSources.forEach(function(input) {
                    if (input.handedness ===  "left") {
                        track.l = toController(input.gamepad!);
                    } else if (input.handedness === "right") {
                        track.r = toController(input.gamepad!);
                    }
                });

                this.props.onTrack(track);
            }
            renderer.render( scene, camera );
        } );
    
        document.body.appendChild( VRButton.createButton( renderer ) );
    }

    componentDidUpdate() {
        this.left.current!.srcObject = this.props.left ?? null;
        this.right.current!.srcObject = this.props.right ?? null;
    }
}

function toController(game: Gamepad): Controller {
    //I am just guessing
    return {
        x: game.axes[0],
        y: game.axes[1],
        a: game.buttons[0].pressed,
        b: game.buttons[1].pressed,
        c: game.buttons[2].value,
        d: game.buttons[3].value,
    }
}