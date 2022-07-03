export default class Status {
    constructor(private elem: HTMLElement){
    }

    status(message: string) {
        this.elem.innerText = message;
        this.elem.className = ""
    }

    warn(message: string) {
        this.elem.innerText = message;
        this.elem.className = "warn";
    }

    error(message: string) {
        this.elem.innerText = message;
        this.elem.className = "error";
    }
}