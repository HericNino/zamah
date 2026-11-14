import { readFileSync } from 'node:fs';
import { defineConfig } from 'vitest/config';

// The sandbox shows a GitHub link when package.json has a `repository` field.
const pkg = JSON.parse(readFileSync(new URL('./package.json', import.meta.url), 'utf8')) as {
  repository?: string | { url?: string };
};
const repository = typeof pkg.repository === 'string' ? pkg.repository : (pkg.repository?.url ?? '');
const repoUrl = repository.replace(/^git\+/, '').replace(/\.git$/, '');

export default defineConfig({
  // Relative asset paths, so the build works from any sub-path (e.g. GitHub Pages).
  base: './',
  define: {
    __REPO_URL__: JSON.stringify(repoUrl),
  },
  build: {
    target: 'es2022',
  },
  test: {
    include: ['tests/**/*.test.ts'],
    // Generous, because the stability and fuzz tests are CPU-bound and CI machines are slow.
    testTimeout: 30_000,
  },
});
