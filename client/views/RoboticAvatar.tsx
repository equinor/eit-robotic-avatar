import React from "react";

import styled, { createGlobalStyle } from "styled-components";
import Module from "./Module";
import Model from '../models/robotic-avatar'
import Video from "./Video";
import Viewport from "./Viewport";

const GlobalStyle = createGlobalStyle`
    html, body, #robotic_avatar {
        margin: 0;
        height: 100%;
    }
`

const Layout = styled.main`
    height: 100%;
    display: grid;
    box-sizing: border-box;
    grid-template-columns: 3fr 2fr 5fr 2fr 3fr;
    grid-template-rows: 2fr 1fr;
    grid-template-areas: 
        "left-video view view view right-video"
        "neck neck network car car";
    gap: 16px 16px;
    background-color: rgb(220, 220, 220);
    padding: 8px;
`

const LeftVideo = styled(Video)`
     grid-area: left-video;
`
const View = styled(Viewport)`
    grid-area: view;
`
const RightVideo = styled(Video)`
    grid-area: right-video;
`
const Neck = styled(Module)`
    grid-area: neck;
`
const Network = styled(Module)`
    grid-area: network;
`
const Car = styled(Module)`
    grid-area: car;
`
interface RoboticAvatarProps {
    model: Model
}


export default class RoboticAvatar extends React.Component<RoboticAvatarProps> {
    render(): React.ReactNode {
        return <Layout>
            <GlobalStyle/>
            <LeftVideo title="Left Video Setting" model={this.props.model.left} />
            <View/>
            <RightVideo title="Right Video Setting" model={this.props.model.right} />

            <Neck title="Neck Robot Setting" status="error" message="Not Implemented">Not Implemented</Neck>
            <Network title="Networking Settings" status="error" message="Not Implemented">Not Implemented</Network>
            <Car title="Car Settings" status="error" message="Not Implemented">Not Implemented</Car>
        </Layout>
    }
}