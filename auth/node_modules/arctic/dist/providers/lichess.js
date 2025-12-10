import { createS256CodeChallenge } from "../oauth2.js";
import { createOAuth2Request, sendTokenRequest } from "../request.js";
const authorizationEndpoint = "https://lichess.org/oauth";
const tokenEndpoint = "https://lichess.org/api/token";
export class Lichess {
    clientId;
    redirectURI;
    constructor(clientId, redirectURI) {
        this.clientId = clientId;
        this.redirectURI = redirectURI;
    }
    createAuthorizationURL(state, codeVerifier, scopes) {
        const url = new URL(authorizationEndpoint);
        url.searchParams.set("response_type", "code");
        url.searchParams.set("client_id", this.clientId);
        url.searchParams.set("state", state);
        url.searchParams.set("scope", scopes.join(" "));
        url.searchParams.set("redirect_uri", this.redirectURI);
        const codeChallenge = createS256CodeChallenge(codeVerifier);
        url.searchParams.set("code_challenge_method", "S256");
        url.searchParams.set("code_challenge", codeChallenge);
        return url;
    }
    async validateAuthorizationCode(code, codeVerifier) {
        const body = new URLSearchParams();
        body.set("grant_type", "authorization_code");
        body.set("code", code);
        body.set("code_verifier", codeVerifier);
        body.set("redirect_uri", this.redirectURI);
        const request = createOAuth2Request(tokenEndpoint, body);
        const tokens = await sendTokenRequest(request);
        return tokens;
    }
}
