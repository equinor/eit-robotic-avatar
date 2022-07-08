import { Typography } from "@equinor/eds-core-react"
import { ReactNode } from "react"
import styled from "styled-components"


interface ModuleProp {
    className?: string
    children: ReactNode,
    title: string,
    status: "ok" | "warn" | "error",
    message: string,
}

const Grid = styled.section`
    display: grid;
    grid-template-rows: min-content auto min-content;
    background-color: rgb(255, 255, 255);
`

const Container = styled.section`
    position: relative;
`

const Status = styled(Typography)`
    & {
        padding: 8px;
    }
    &.warn {
        background-color: rgb(255, 231, 214);
    }
    &.error {
        background-color: rgb(255, 193, 193);
    }
`

export default function Module(props: ModuleProp) {
    return <Grid className={props.className}>
        <Typography variant="h4" as="header">{props.title}</Typography>
        <Container>{props.children}</Container>
        <Status className={props.status} variant="body_short"> {props.message} </Status>
    </Grid>
}