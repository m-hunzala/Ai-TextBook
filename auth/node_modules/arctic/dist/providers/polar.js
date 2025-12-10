import { createOAuth2Request, sendTokenRequest, sendTokenRevocationRequest } from "../request.js";
import { createS256CodeChallenge } from "../oauth2.js";
const authorizationEndpoint = "https://polar.sh/oauth2/authorize";
const tokenEndpoint = "https://api.polar.sh/v1/oauth2/token";
const tokenRevocationEndpoint = "https://api.polar.sh/v1/oauth2/revoke";
export class Polar {
    clientId;
    clientSecret;
    redirectURI;
    constructor(clientId, clientSecret, redirectURI) {
        this.clientId = clientId;
        this.clientSecret = clientSecret;
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
        body.set("client_id", this.clientId);
        body.set("client_secret", this.clientSecret);
        const request = createOAuth2Request(tokenEndpoint, body);
        const tokens = await sendTokenRequest(request);
        return tokens;
    }
    async refreshAccessToken(refreshToken) {
        const body = new URLSearchParams();
        body.set("grant_type", "refresh_token");
        body.set("refresh_token", refreshToken);
        body.set("client_id", this.clientId);
        body.set("client_secret", this.clientSecret);
        const request = createOAuth2Request(tokenEndpoint, body);
        const tokens = await sendTokenRequest(request);
        return tokens;
    }
    async revokeToken(token) {
        const body = new URLSearchParams();
        body.set("token", token);
        body.set("client_id", this.clientId);
        body.set("client_secret", this.clientSecret);
        const request = createOAuth2Request(tokenRevocationEndpoint, body);
        await sendTokenRevocationRequest(request);
    }
}
