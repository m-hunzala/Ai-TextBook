import type { OAuth2Tokens } from "../oauth2.js";
export declare class Lichess {
    private clientId;
    private redirectURI;
    constructor(clientId: string, redirectURI: string);
    createAuthorizationURL(state: string, codeVerifier: string, scopes: string[]): URL;
    validateAuthorizationCode(code: string, codeVerifier: string): Promise<OAuth2Tokens>;
}
