import React from "react";
import Status from "./src/status";
import Video from "./src/video";

import "./index.css"

export class RoboticAvatar extends React.Component {
    private left_video: React.RefObject<HTMLElement> = React.createRef();
    private left_video_status: React.RefObject<HTMLElement> = React.createRef();
    private right_video: React.RefObject<HTMLElement> = React.createRef();
    private right_video_status: React.RefObject<HTMLElement> = React.createRef();

    render(): React.ReactNode {
        return <>
            <section className="left_video">
                <header>Left Video Settings</header>
                <section ref={this.left_video}> Loading </section>
                <footer ref={this.left_video_status} className="error">Not Loaded</footer>
            </section>
            <section className="view">
                <header>Preview Viewport</header>
                <section id="viewport"> Not Implemented </section>
                <footer id="viewport_status" className="error"> Not Implemented</footer>
            </section>
            <section className="right_video">
                <header>Right Video Settings</header>
                <section ref={this.right_video}> Loading </section>
                <footer ref={this.right_video_status} className="error"> Not Loaded </footer>
            </section>
            <section className="neck">
                <header>Neck Robot Settings</header>
                <section id="neck"> Not Implemented </section>
                <footer id="neck_status" className="error"> Not Implemented</footer>
            </section>
            <section className="network">
                <header>Networking Settings</header>
                <section id="network"> Not Implemented </section>
                <footer id="network_status" className="error"> Not Implemented</footer>
            </section>
            <section className="car">
                <header>Car Settings</header>
                <section id="network"> Not Implemented </section>
                <footer id="network_status" className="error"> Not Implemented</footer>
            </section>
        </> 
    }

    componentDidMount() {
        const left_video_status = new Status(this.left_video_status.current!);
        const left_video = new Video(this.left_video.current!, left_video_status);

        const right_video_status = new Status(this.right_video_status.current!);
        const right_video = new Video(this.right_video.current!, right_video_status)
    }
}