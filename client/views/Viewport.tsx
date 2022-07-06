import { Button } from "@equinor/eds-core-react";
import React from "react";
import styled from "styled-components";
import { Mesh, MeshBasicMaterial, OrthographicCamera, PlaneGeometry, Scene, Vector2, VideoTexture, WebGLRenderer } from "three";
import Module from "./Module";

const Canvas = styled.canvas`
    background-color: #000;
    height: 100%;
    width: 100%;
`

const HiddenVideo = styled.video`
    display: none;
`

const Overlay = styled.div`
    position: absolute;
    bottom: 0;
    left: 0;
    right: 0;
    text-align: center;
`

interface Props {
    className?: string
}

interface State {
    fps: number
}

export default class Viewport extends React.Component<Props, State>{
    private canvas: React.RefObject<HTMLCanvasElement>;
    private left: React.RefObject<HTMLVideoElement>;
    private right: React.RefObject<HTMLVideoElement>;
    private fps: number = 0;
    private seconds: number = 0;

    constructor(props: Props) {
        super(props);

        this.state = {
            fps: 0
        }

        this.canvas = React.createRef<HTMLCanvasElement>();
        this.left = React.createRef<HTMLVideoElement>();
        this.right = React.createRef<HTMLVideoElement>();
    }

    componentDidMount() {
        const scene = new Scene();

        const camera = new OrthographicCamera( -1, 1, 1, -1, -1, 1  );

        const renderer = new WebGLRenderer({
            canvas: this.canvas.current!
        });

        renderer.setSize(3664, 1920, false); 

        const textureLeft = new VideoTexture( this.left.current! );
        textureLeft.center = new Vector2( 0.5, 0.5 )
        textureLeft.rotation = 90  * (Math.PI/180);

        const left = new Mesh(
            new PlaneGeometry( 1, 2 ),
            new MeshBasicMaterial( { map: textureLeft } )
        );
        left.position.x = -0.5;
        scene.add( left );

        const textureRight= new VideoTexture( this.right.current! );
        textureRight.center = new Vector2( 0.5, 0.5 )
        textureRight.rotation = 90 * 3 * (Math.PI/180);
        const right = new Mesh(
            new PlaneGeometry( 1, 2 ),
            new MeshBasicMaterial( { map: textureRight } )
        );
        right.position.x = 0.5;
        scene.add( right );

        renderer.setAnimationLoop( (time) => {
            this.fps += 1;
            const sec = Math.floor(time/1000);
            if (sec > this.seconds) {
                this.setState({
                    fps: this.fps
                })
                this.seconds = sec;
                this.fps = 0;
            }
            renderer.render( scene, camera );
        } );
    }

    render(): React.ReactNode {
        const msg = `Rendering at: 3664x1920 with ${this.state.fps} frames per second`

        return <Module className={this.props.className} title="Preview Viewport" status={"ok"} message={msg}>
            <Canvas ref={this.canvas}/>
            <Overlay>
                <Button disabled>Fullscreen</Button>
                <Button disabled>Open VR</Button>
            </Overlay>
            <HiddenVideo ref={this.left}/>
            <HiddenVideo ref={this.right}/>
        </Module>
    }
}