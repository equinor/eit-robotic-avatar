import { Button, Card, Divider, Typography } from "@equinor/eds-core-react";
import React from "react";
import status from "./status";

interface VideoProp{
    title: string
}

export default class Video extends React.Component<VideoProp>{
    render(): React.ReactNode {
        return <Card>
        <Card.Header>
            <Card.HeaderTitle>
                <Typography variant="h4">{this.props.title}</Typography>
            </Card.HeaderTitle>
        </Card.Header>
        <Card.Content>
            <Typography variant="h5">Title</Typography>
            <Typography variant="body_short">
                Lorem ipsum dolor sit amet, consectetur adipiscing elit.
            </Typography>
            <Divider style={{ width: '100%' }} />
        </Card.Content>
        <Card.Content>
        </Card.Content>
        <Card.Actions>
            <Typography variant="body_short">
                Lorem ipsum dolor sit amet, consectetur adipiscing elit.
            </Typography>
        </Card.Actions>
    </Card>
    }
}