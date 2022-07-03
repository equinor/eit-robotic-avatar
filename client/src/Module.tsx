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

const ModuleGrid = styled.section`
    display: grid;
    grid-template-rows: min-content auto min-content;
    background-color: rgb(255, 255, 255);
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
    return <ModuleGrid className={props.className}>
        <Typography variant="h4" as="header">{props.title}</Typography>
        <section>{props.children}</section>
        <Status className={props.status} variant="body_short"> {props.message} </Status>
    </ModuleGrid>
}