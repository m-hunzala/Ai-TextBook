import * as _better_fetch_fetch from '@better-fetch/fetch';

type ConsoleEsque = {
    log: (...args: any[]) => void;
    error: (...args: any[]) => void;
    success?: (...args: any[]) => void;
    fail?: (...args: any[]) => void;
    warn?: (...args: any[]) => void;
};
interface LoggerOptions {
    /**
     * Enable or disable the logger
     * @default true
     */
    enabled?: boolean;
    /**
     * Custom console object
     */
    console?: ConsoleEsque;
    /**
     * Enable or disable verbose mode
     */
    verbose?: boolean;
}
declare const logger: (options?: LoggerOptions) => {
    id: string;
    name: string;
    version: string;
    hooks: {
        onRequest<T extends Record<string, any>>(context: _better_fetch_fetch.RequestContext<T>): void;
        onSuccess(context: _better_fetch_fetch.SuccessContext<any>): Promise<void>;
        onRetry(response: _better_fetch_fetch.ResponseContext): void;
        onError(context: _better_fetch_fetch.ErrorContext): Promise<void>;
    };
};

export { type LoggerOptions, logger };
