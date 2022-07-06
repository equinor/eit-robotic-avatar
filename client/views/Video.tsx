import { Autocomplete, Button, Input, Label, NativeSelect, Slider, Switch, Typography } from "@equinor/eds-core-react";
import React from "react";
import Model, { VideoSource } from "../models/video"
import Module from "./Module";

interface VideoProps {
    model: Model,
    title: string,
}

interface VideoState {
    sources: VideoSource[],
    connected: boolean;
}

export default class Video extends React.Component<VideoProps,VideoState> {
    constructor(props: VideoProps) {
        super(props);
        this.state = {
            sources: [],
            connected: this.props.model.show_local
        };
    }

    connectedSwitch = (e: React.ChangeEvent<HTMLInputElement>) => {
        this.props.model.show_local = !this.props.model.show_local;
        this.setState({
            sources: [],
            connected: this.props.model.show_local
        })
        this.updateSources();
    };

    updateSources() {
        this.props.model.getSources().then((sources) => {
            this.setState({
                sources: sources
            })
        })
    }

    render(): React.ReactNode {
        const model = this.props.model;

        return <Module title={this.props.title} status={"error"} message={"No Video Source Configured"}>
            <Typography variant="h5">Video Source</Typography>
            <Switch label="Show connected sources" onChange={this.connectedSwitch} checked={model.show_local} />
            <Switch label="Show remote sources" disabled />
            <Autocomplete label="Video sources:" options={this.state.sources} optionLabel={(source) => source.label}/>
            <Label label="Resolution Height:" />
            <Input type="number" disabled/>
            <Label label="Resolution Width:" />
            <Input type="number" disabled/>
            <Label label="Frame rate:" />
            <Input type="number" disabled/>
            <Button disabled>Apply Resolution Changes</Button>
            <Typography variant="h5">View Settings</Typography>
            <Autocomplete label="Rotate Video" options={["0°", "90°", "180°", "270°"]} initialSelectedOptions={["0°"]} disabled/>
            <Label label="Zoom: (%)" />
            <Slider value={100} min={50} max={200} aria-label="Zoom in %" disabled/>
        </Module>
    }
}