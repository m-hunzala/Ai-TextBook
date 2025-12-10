import { createS256CodeChallenge } from "../oauth2.js";
import { createOAuth2Request, encodeBasicCredentials, sendTokenRequest, sendTokenRevocationRequest } from "../request.js";
export class Authentik {
    authorizationEndpoint;
    tokenEndpoint;
    tokenRevocationEndpoint;
    clientId;
    clientSecret;
    redirectURI;
    constructor(domain, clientId, clientSecret, redirectURI) {
        this.authorizationEndpoint = `https://${domain}/application/o/authorize/`;
        this.tokenEndpoint = `https://${domain}/application/o/token/`;
        this.tokenRevocationEndpoint = `https://${domain}/application/o/revoke`;
        this.clientId = clientId;
        this.clientSecret = clientSecret;
        this.redirectURI = redirectURI;
    }
    createAuthorizationURL(state, codeVerifier, scopes) {
        const url = new URL(this.authorizationEndpoint);
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
        const request = createOAuth2Request(this.tokenEndpoint, body);
        const encodedCredentials = encodeBasicCredentials(this.clientId, this.clientSecret);
        request.headers.set("Authorization", `Basic ${encodedCredentials}`);
        const tokens = await sendTokenRequest(request);
        return tokens;
    }
    async refreshAccessToken(refreshToken) {
        const body = new URLSearchParams();
        body.set("grant_type", "refresh_token");
        body.set("refresh_token", refreshToken);
        const request = createOAuth2Request(this.tokenEndpoint, body);
        const encodedCredentials = encodeBasicCredentials(this.clientId, this.clientSecret);
        request.headers.set("Authorization", `Basic ${encodedCredentials}`);
        const tokens = await sendTokenRequest(request);
        return tokens;
    }
    async revokeToken(token) {
        const body = new URLSearchParams();
        body.set("token", token);
        const request = createOAuth2Request(this.tokenRevocationEndpoint, body);
        const encodedCredentials = encodeBasicCredentials(this.clientId, this.clientSecret);
        request.headers.set("Authorization", `Basic ${encodedCredentials}`);
        await sendTokenRevocationRequest(request);
    }
}
