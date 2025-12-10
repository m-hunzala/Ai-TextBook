// @ts-check

import fs from 'fs';
import path from 'path';
import { toString } from 'mdast-util-to-string';
import { remark } from 'remark';
import { read } from 'to-vfile';
import { unified } from 'unified';
import remarkParse from 'remark-parse';

/** @type {import('@docusaurus/types').PluginModule} */
const RawContentPlugin = (context, options) => {
  return {
    name: 'docusaurus-plugin-raw-content',
    
    async loadContent() {
      // This function would load raw content for all docs
      // For performance reasons, we'll implement a more targeted approach
      return {}; // Placeholder
    },
    
    configureWebpack(config, isServer, utils) {
      return {
        resolve: {
          alias: {
            '@site/content': path.resolve(__dirname, '../../docs'),  // Adjust path as needed
          }
        }
      };
    }
  };
};

export default RawContentPlugin;