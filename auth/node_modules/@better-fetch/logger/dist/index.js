var __defProp = Object.defineProperty;
var __getOwnPropSymbols = Object.getOwnPropertySymbols;
var __hasOwnProp = Object.prototype.hasOwnProperty;
var __propIsEnum = Object.prototype.propertyIsEnumerable;
var __defNormalProp = (obj, key, value) => key in obj ? __defProp(obj, key, { enumerable: true, configurable: true, writable: true, value }) : obj[key] = value;
var __spreadValues = (a, b) => {
  for (var prop in b || (b = {}))
    if (__hasOwnProp.call(b, prop))
      __defNormalProp(a, prop, b[prop]);
  if (__getOwnPropSymbols)
    for (var prop of __getOwnPropSymbols(b)) {
      if (__propIsEnum.call(b, prop))
        __defNormalProp(a, prop, b[prop]);
    }
  return a;
};

// src/index.ts
import { createConsola } from "consola";

// src/util.ts
function getStatusText(status) {
  switch (status) {
    case 100:
      return "Continue";
    case 101:
      return "Switching Protocols";
    case 102:
      return "Processing";
    case 200:
      return "OK";
    case 201:
      return "Created";
    case 202:
      return "Accepted";
    case 203:
      return "Non-Authoritative Information";
    case 204:
      return "No Content";
    case 205:
      return "Reset Content";
    case 206:
      return "Partial Content";
    case 207:
      return "Multi-Status";
    case 208:
      return "Already Reported";
    case 226:
      return "IM Used";
    case 300:
      return "Multiple Choices";
    case 301:
      return "Moved Permanently";
    case 302:
      return "Found";
    case 303:
      return "See Other";
    case 304:
      return "Not Modified";
    case 305:
      return "Use Proxy";
    case 307:
      return "Temporary Redirect";
    case 308:
      return "Permanent Redirect";
    case 400:
      return "Bad Request";
    case 401:
      return "Unauthorized";
    case 402:
      return "Payment Required";
    case 403:
      return "Forbidden";
    case 404:
      return "Not Found";
    case 405:
      return "Method Not Allowed";
    case 406:
      return "Not Acceptable";
    case 407:
      return "Proxy Authentication Required";
    case 408:
      return "Request Timeout";
    case 409:
      return "Conflict";
    case 410:
      return "Gone";
    case 411:
      return "Length Required";
    case 412:
      return "Precondition Failed";
    case 413:
      return "Payload Too Large";
    case 414:
      return "URI Too Long";
    case 415:
      return "Unsupported Media Type";
    case 416:
      return "Range Not Satisfiable";
    case 417:
      return "Expectation Failed";
    case 418:
      return "I'm a teapot";
    case 421:
      return "Misdirected Request";
    case 422:
      return "Unprocessable Entity";
    case 423:
      return "Locked";
    case 424:
      return "Failed Dependency";
    case 425:
      return "Too Early";
    case 426:
      return "Upgrade Required";
    case 428:
      return "Precondition Required";
    case 429:
      return "Too Many Requests";
    case 431:
      return "Request Header Fields Too Large";
    case 451:
      return "Unavailable For Legal Reasons";
    case 500:
      return "Internal Server Error";
    case 501:
      return "Not Implemented";
    case 502:
      return "Bad Gateway";
    case 503:
      return "Service Unavailable";
    case 504:
      return "Gateway Timeout";
    case 505:
      return "HTTP Version Not Supported";
    case 506:
      return "Variant Also Negotiates";
    case 507:
      return "Insufficient Storage";
    case 508:
      return "Loop Detected";
    case 510:
      return "Not Extended";
    case 511:
      return "Network Authentication Required";
    default:
      return "Unknown";
  }
}

// src/index.ts
var c = createConsola({
  fancy: true,
  formatOptions: {
    columns: 80,
    colors: true,
    compact: 10,
    date: false
  }
});
var defaultConsole = {
  error(...args) {
    c.error("", ...args);
  },
  log(...args) {
    c.info("", ...args);
  },
  success(...args) {
    c.success("", ...args);
  },
  fail(...args) {
    c.fail("", ...args);
  },
  warn(...args) {
    c.warn("", ...args);
  }
};
var logger = (options) => {
  const opts = __spreadValues({
    console: defaultConsole,
    enabled: true
  }, options);
  const { enabled } = opts;
  return {
    id: "logger",
    name: "Logger",
    version: "1.0.0",
    hooks: {
      onRequest(context) {
        if (!enabled) return;
        opts.console.log("Request being sent to:", context.url.toString());
      },
      async onSuccess(context) {
        if (!enabled) return;
        const log = opts.console.success || opts.console.log;
        log("Request succeeded", context.data);
      },
      onRetry(response) {
        if (!enabled) return;
        const log = opts.console.warn || opts.console.log;
        log(
          "Retrying request...",
          "Attempt:",
          (response.request.retryAttempt || 0) + 1
        );
      },
      async onError(context) {
        if (!enabled) return;
        const log = opts.console.fail || opts.console.error;
        let obj;
        try {
          if (opts.verbose) {
            const res = context.response.clone();
            const json = await res.json();
            if (json) {
              obj = json;
            }
          }
        } catch (e) {
        }
        log(
          "Request failed with status: ",
          context.response.status,
          `(${context.response.statusText || getStatusText(context.response.status)})`
        );
        (options == null ? void 0 : options.verbose) && obj && opts.console.error(obj);
      }
    }
  };
};
export {
  logger
};
//# sourceMappingURL=index.js.map