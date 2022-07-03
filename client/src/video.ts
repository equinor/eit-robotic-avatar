import status from "./status";

export default class Video{
    constructor(private elem: HTMLElement, private status: status) {
        this.status.error("No video source configured!");
    }
}