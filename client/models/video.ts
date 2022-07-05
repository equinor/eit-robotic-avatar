import { ThemeConsumer } from "styled-components";

export default class Video{
    public onChange: () => void = noop;

    private _show_local = false;

    get show_local(): boolean {
        return this._show_local;
    }

    set show_local(show: boolean) {
        if (this._show_local !== show){
            this._show_local = show;
            this.onChange();
        }
    }
}

function noop(){

}