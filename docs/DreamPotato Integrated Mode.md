# DreamPotato Integrated Mode

We want to make Hollycast a "one stop shop" for emulating the Dreamcast and VMU together.

The goal is that a user can just launch Hollycast, enable this mode, and have the VMU emulator appear, and use the correct VMU file automatically, based on what Hollycast is doing. This removes the need to manually launch DreamPotato, manually select a file for the particular game, and so on.

We refer to this functionality as "integrated mode". The existing feature where both DreamPotato and Hollycast are launched manually, is called "standalone mode".

## New command line flags

We add the following command line flags to DreamPotato:

- `--integrated`: Enables integrated mode in DreamPotato. This disables a number of UI commands and settings. Mainly, commands related to managing VMU files.
- `--tcp-port 12345`: The TCP port that DreamPotato will connect to upon startup.
- `--port A`: The port letter that the DreamPotato instance should use. Any port letters A-D are permitted.
- `--slots 1`: Indicates which slots contain "DreamPotato VMUs". Slots `1`, `2`, `both` are permitted.

When the integrated mode is enabled in Hollycast, then, Hollycast will launch a DreamPotato subprocess at the appropriate time, passing the appropriate values for the above flags.

## TCP connection

In standalone mode, DreamPotato acts as the TCP server in the connection, using a hard-coded set of local ports. This was mainly because the DreamConn connectivity was the starting point for this work, and in that case DreamConn was the server.

Now with integrated mode, Hollycast, as the parent process, acts as the server instead. This way, we can let the OS just assign a TCP port number automatically, pass that port number to the child process via command line, and establish the connection in a more straightforward and predictable way.
- This also lets many instances of Hollycast "just work" with many instances of DreamPotato, without any of them stomping on each other.
- This also lets DreamPotato know more consistently when it should exit, even if Hollycast crashes or similar.

## File selection

We essentially want the "VMU file used by Hollycast", to be automatically provided to DreamPotato.

This is accomplished when first launching DreamPotato, by simply passing it the path of the VMU file as it already supports today.

However, the VMU path can change depending on the game (Per-game VMU A1). We don't want to re-launch the DreamPotato process when changing games as this is disruptive and slow.

To support this, we add the following Maple pseudo-commands:
- `Open File`: Contains a file path as the payload. DreamPotato will receive this message and will open the given file using the "addressed" VMU.
    - Command code: 0xC0.
    - Since Maple payloads are 32-bit words, the file path may be zero-padded at the end. DreamPotato will strip the trailing zeros off of the path before using it.
    - The file path is assumed to be encoded in UTF-8.

## Save states

We aren't doing anything special with this right now, compared to the existing DreamPotato connectivity.

For example, loading a save state in this mode, will not revert the VMU contents to when the state was saved, like would be done for ordinary VMUs. Instead, the VMU is automatically reconnected to the Dreamcast if any of the already-read blocks have changed since the state was saved. This allows the game to observe changes in the VMU contents, but, can cause disruptive behaviors in games. For example, in Sonic Adventure 2, the game will stop briefly and show a popup.

Theoretically, this tighter integration, would be an opportunity to ensure that the DreamPotato savestate is strongly coupled to the Hollycast savestate. This would allow restoring the VMU contents to match the savestate under all conditions, and prevent a need to "simulate reconnecting" VMUs. However, this is additional work and not thought to be part of the "most critical path" to get working.

Note that the semantics of a `maple_sega_vmu` after loading state are:
- Replace the `flash_data` with the data from the savestate.
- Mark `fullSaveNeeded = true`.
- When a flash write request comes through, observe the `fullSaveNeeded` and write the whole VMU content to disk instead of partial content.

This means that the VMU files on disk only change if you load a state and then save a file in-game afterwards. There are multiple possible reasons for this:
- Loading a state may be just a temporary "poke around" type of activity. If the user never saves a game, then it's likely they don't want their VMU files replaced.
- However some games autosave just from completing a level, collecting an item, etc. So the user might end up saving and overwriting the old file by accident.

This behavior can make it hard for users to predict what effects loading a state might have. They may not realize that loading a state can overwrite their save files, but only if they save to the VMU later on. This seems problematic.

The problem is related to the issue with device settings UI after loading state. The connected maple devices might be completely different from what appears in the UI, because a state was loaded from when a previous configuration was being used. But nothing in the UI tells you that what it's currently showing you is not what's actually being used.

We might want to adjust the default behavior of the `maple_sega_vmu` as well as the integrated DreamPotato VMU to see if we can accomplish all of the following at the same time:
- No disruptive behaviors when loading state (e.g. automatic reconnect due to content difference).
- Behavior is easier for user to predict.
- No possibility of accidental data loss.
  - For example, require some extra/more explicit action, before either the pre-load-state or post-load-state save data could be lost.

However, it's not clear what design would actually accomplish all of the above.

One approach for implementing the current `maple_sega_vmu` semantics for integrated DreamPotato VMUs would be:
1) When saving state, ensure all the DreamPotato flash data is properly mirrored back to Hollycast. (e.g. changes made while VMU was ejected are recorded).
2) Add a new custom maple command, to tell DP to WriteBlock but not mirror the data to disk. Have DP set its own equivalent of the `fullSaveNeeded` flag.
3) Write all the flash data to DP from the savestate using the new command.
4) Use another new custom maple command to set the docked/ejected state to match whatever it was when the state was saved.

It's also not clear if we should overwrite VMU contents or state, if the DreamPotato VMU was ejected at the time the state was saved. 2 options that feel reasonable:
- Just ensure it's still ejected when loading state. Don't restore the contents as the Dreamcast game was not tracking this anyway.
- Alternatively, serialize an entire DreamPotato save state into the Hollycast save state, and let DreamPotato load it. This probably involves more custom maple commands.
    - Otherwise, if we only restored the flash content, we'd need to also reset the VMU, which would just be another form of disruptive behavior.
