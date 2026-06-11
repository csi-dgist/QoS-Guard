Visual Studio Code 1.117

Show release notes after an update

Follow us on LinkedIn, X, Bluesky | View online

Release date: April 22, 2026

Welcome to the 1.117 release of Visual Studio Code. This release adds new capabilities for Copilot Enterprise and Business users and further improves the agent experience in VS Code. Here are the highlights for this release:

BYOK for Business and Enterprise: Connect your own API keys for preferred or specialized models directly in VS Code chat.

Incremental chat rendering: Experience more fluid streaming of chat responses.

Terminal improvements: Launch Copilot CLI from any terminal profile.

Happy Coding!

VS Code is rolling out gradually to all users. Use Check for Updates in VS Code to get the latest version immediately.

To try new features as soon as possible, download the nightly Insiders build, which includes the latest updates as soon as they are available.

In this update
GitHub Copilot
Chat experience
Agent experience
Terminal
Languages
Deprecated features and settings
Thank you
Bring your own key for Copilot Business and Enterprise
Teams often need specific models for compliance, performance, or cost reasons, but switching between tools to use them slows developers down. Bring your own language model key (BYOK) lets Copilot Business and Enterprise users connect their own API keys for providers like OpenRouter, Ollama, Google, OpenAI, and more, so they can use those models directly in VS Code chat.

By default, BYOK is enabled and administrators can disable it with the Bring Your Own Language Model Key policy in the Copilot policy settings on GitHub.com. This gives administrators control over which model providers are available to their organization while keeping developers in their existing workflow.

After the policy is enabled, organization members can add models from built-in providers or install language model provider extensions.

Chat experience
Incremental rendering of chat responses (Experimental)
Chat responses feel more fluid and natural with incremental rendering, which streams content block-by-block with optional animations as tokens arrive. Instead of the default timer-based rendering, this experimental approach renders each block as it becomes ready, reducing the perceived wait time for longer responses.

Configure incremental response rendering with the following settings:

  chat.experimental.incrementalRendering.enabled : Enable or disable incremental response rendering with optional block-level animation when streaming chat responses. Default: true.
  chat.experimental.incrementalRendering.animationStyle : Configure the animation style for incremental response rendering. Options: none, fade, rise, blur, scale, slide, reveal. Default: fade.
  chat.experimental.incrementalRendering.buffering : Configure how content is buffered before rendering during incremental response rendering. Lower buffering levels render faster but may show incomplete sentences or partially-formed Markdown. Options: off, word, paragraph. Default: word.
Sort agent sessions by recent activity
When you accumulate many agent sessions, finding the right one can be difficult. The Agent Sessions view supports sorting sessions by when they were created or last updated, so you can quickly pick up where you left off.

Screenshot of the Chat view with the filter context menu open, showing the sort by updated/created actions.

System notifications for background terminal commands
When an agent runs a long-running terminal command in the background, it can be easy to lose track of its progress. These commands now surface as System notifications in the chat response, so you can monitor their status without switching to the terminal.

Screenshot of a system notification appearing in the chat response.

Agent experience
Visual Studio Code Agents (Insiders)
Note: The Visual Studio Code Agents app is currently in preview and only available when installing VS Code Insiders.

The Visual Studio Code Agents app is a companion app that ships alongside VS Code Insiders, providing a focused, agent-native environment where you can run parallel sessions across repos, review diffs inline, and iterate on multi-step coding tasks. Introduced in 1.115, the app continues to evolve based on feedback.

Updates in this release:

Create sub-sessions: Select + in the session title to spawn a sub-session from the current session. This is handy for starting additional work in context, such as parallel research or a code review, without losing your place in the parent session.
Inline change rendering: Improvements to how changes are rendered inline make it easier to scan and compare diffs when the agent edits your code.
Update experience: Improvements to the update flow across operating systems make it smoother to stay on the latest version.
Theming, chat response, and UX polish: Continued refinements to theming, session list and response rendering, and overall UX across the app.
Screenshot of the VS Code Agents - Insiders app with proposed changes.

As in previous releases, you can open the app via the same methods:

Launch Visual Studio Code Agents - Insiders from your Start menu or Applications folder in the OS.
Run Chat: Open Agents Application from the VS Code Insiders Command Palette.
Select Try out the new Agents app from the VS Code Insiders welcome page.
Terminal
Launch Copilot CLI with a custom terminal profile
The Copilot CLI terminal profile can now be launched from the terminal panel, even when your default terminal profile is set to a non-default shell, such as fish on macOS or Linux, or Git Bash on Windows.

Previously, selecting GitHub Copilot CLI from the terminal profile picker in this configuration produced a No terminal profile options provided for id 'copilot-cli' error and the terminal failed to start.

Terminal title for agent CLIs
Agent CLIs like Copilot CLI, Claude Code, and Gemini CLI typically run as node processes, which meant the terminal title showed a generic node label. This made it hard to tell which agent was running in each terminal. The terminal now detects these agent CLIs as a distinct shell type and uses the OSC title sequence emitted by the CLI as the terminal title, so each terminal clearly identifies the agent it is hosting.

Screenshot showing the terminal title reflecting the running agent CLI via its OSC title sequence.

The improved detection covers Copilot CLI, Claude Code, and Gemini CLI on macOS, Linux, and Windows. Codex is not yet detected on macOS because it does not currently emit an OSC title sequence. This behavior is enabled by default and can be toggled with the   terminal.integrated.tabs.allowAgentCliTitle setting.

Languages
TypeScript 6.0.3
This release includes the TypeScript 6.0.3 recovery release. This minor update fixes a few import bugs and regressions.

Deprecated features and settings
New deprecations in this release
Upcoming deprecations
Thank you
Contributions to our issue tracking:

@gjsjohnmurray (John Murray)
@RedCMD (RedCMD)
@IllusionMH (Andrii Dieiev)
@albertosantini (Alberto Santini)
Contributions to vscode:

@abadawi591 (abadawi-msft): Abadawi/send has image to router PR #308321
@andysharman: fix: default session mode experiment not applying on first session PR #308905
@bocan (Chris Funderburg): Fix crash on null entries in launch.json configurations array PR #308235
@jamestut (James Nugraha): await openEditor in terminal editor split to prevent shadow tab PR #309167
@maruthang (Maruthan G)
fix(tasks): add hover description for required property in taskDefinitions contribution schema (#275670) PR #310764
fix(debug): identify instruction breakpoints by resolved address to allow removal when instructionReference changes (#289678) PR #310763
fix(terminal-chat): dedupe terminal tool-session registrations to prevent listener leak (#309906) PR #310740
fix(chat): guard renderWelcomeViewContentIfNeeded against undisposed input part (#310356) PR #310822
fix: prevent listener leak from duplicate status IDs in language status (#309042) PR #309159
fix(chat): cancel in-flight streaming tool invocations when response is cancelled (#288701) PR #310979
@matts1 (Matt): feat: Support switching to the main window. PR #306573
@NikolaRHristov (Nikola Hristov): fix: make protected members public to resolve mangler build errors PR #310195
@OscarPalafox (Oscar Palafox Verna): Consistent include pathing for new 2026 in theme-defaults PR #309880
@RieBi (Sviatoslav Zubar): Additionaly to newest published version of package also show currently installed version PR #308569
@yogeshwaran-c (Yogeshwaran C)
json: fix language model cache evicting at capacity instead of overflow PR #309176
Do not open debug view on first session start when openDebug is openOnDebugBreak PR #309133
testing: align right-click menu with hover bar on compressed result rows PR #309139
Adopt CodeAction type for built-in css server PR #310055
We really appreciate people trying our new features as soon as they are ready, so check back here often and learn what's new.

If you'd like to read release notes for previous VS Code versions, go to Updates on code.visualstudio.com.

